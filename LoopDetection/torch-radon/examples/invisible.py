import matplotlib.pyplot as plt
import numpy as np
import torch
import time

from torch_radon import Radon, RadonFanbeam
from torch_radon.solvers import cg

from torch_radon.shearlet import ShearletTransform


def circle_mask(size, radius):
    center = (size - 1) / 2
    c0, c1 = np.ogrid[0:size, 0:size]
    return ((c0 - center) ** 2 + (c1 - center) ** 2) <= radius ** 2


def shrink(a, b):
    return (torch.abs(a) - b).clamp_min(0) * torch.sign(a)


def main():
    n_angles = 100
    image_size = 512
    circle_radius = 100
    source_dist = 1.5 * image_size
    batch_size = 1
    n_scales = 5

    angles = (np.linspace(0., 100., n_angles, endpoint=False) - 50.0) / 180.0 * np.pi

    x = np.zeros((image_size, image_size), dtype=np.float32)
    x[circle_mask(image_size, circle_radius)] = 1.0

    radon = Radon(image_size, angles)  # RadonFanbeam(image_size, angles, source_dist)
    shearlet = ShearletTransform(image_size, image_size, [0.5] * n_scales)

    torch_x = torch.from_numpy(x).cuda()
    torch_x = torch_x.view(1, image_size, image_size).repeat(batch_size, 1, 1)
    sinogram = radon.forward(torch_x)

    bp = radon.backward(sinogram)
    sc = shearlet.forward(bp)

    p_0 = 0.02
    p_1 = 0.1
    w = 3 ** shearlet.scales / 400
    w = w.view(1, -1, 1, 1).cuda()

    u_2 = torch.zeros_like(bp)
    z_2 = torch.zeros_like(bp)
    u_1 = torch.zeros_like(sc)
    z_1 = torch.zeros_like(sc)
    f = torch.zeros_like(bp)

    relative_error = []
    start_time = time.time()
    for i in range(100):
        cg_y = p_0 * bp + p_1 * shearlet.backward(z_1 - u_1) + (z_2 - u_2)
        f = cg(lambda x: p_0 * radon.backward(radon.forward(x)) + (1 + p_1) * x, f.clone(), cg_y, max_iter=50)
        sh_f = shearlet.forward(f)

        z_1 = shrink(sh_f + u_1, p_0 / p_1 * w)
        z_2 = (f + u_2).clamp_min(0)
        u_1 = u_1 + sh_f - z_1
        u_2 = u_2 + f - z_2

        relative_error.append((torch.norm(torch_x[0] - f[0]) / torch.norm(torch_x[0])).item())

    runtime = time.time() - start_time
    print("Running time:", runtime)
    print("Running time per image:", runtime / batch_size)
    print("Relative error: ", 100 * relative_error[-1])


with torch.no_grad():
    main()
