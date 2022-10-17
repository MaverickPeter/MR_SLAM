import numpy as np
import torch
from nose.tools import assert_equal
from parameterized import parameterized
import random

from .utils import random_symbolic_function, symbolic_discretize, symbolic_forward, TestHelper

from torch_radon.volumes import Volume2D
import torch_radon as tr

random.seed(42)
device = torch.device('cuda')
test_helper = TestHelper("parallel_beam")

# (batch_size, angles, volume, spacing, det_count)
params = []

# check different batch sizes
for batch_size in [1, 3, 17, 32]:
    params.append((batch_size, (0, np.pi, 128), None, 1.0, 128))

# check few and many angles which are not multiples of 16
for angles in [(0, np.pi, 19), (0, np.pi, 803)]:
    params.append((4, angles, None, 1.0, 128))

# change volume size
for height, width in [(128, 256), (256, 128), (75, 149), (81, 81)]:
    s = max(height, width)
    volume = Volume2D()
    volume.set_size(height, width)
    params.append((4, (0, np.pi, 64), volume, 2.0, s))

# change volume scale and center
for center in [(0, 0), (17, -25), (53, 49)]:
    for voxel_size in [(1, 1), (0.75, 0.75), (1.5, 1.5), (0.7, 1.3), (1.3, 0.7)]:
        det_count = int(179 * max(voxel_size[0], 1) * max(voxel_size[1], 1) * np.sqrt(2))
        volume = Volume2D(center, voxel_size)
        volume.set_size(179, 123)
        params.append((4, (0, np.pi, 128), volume, 2.0, det_count))

for spacing in [1.0, 0.5, 1.3, 2.0]:
    for det_count in [79, 128, 243]:
        for src_dist, det_dist in [(128, 128), (64, 128), (128, 64), (503, 503)]:
            volume = Volume2D()
            volume.set_size(128, 128)
            params.append((4, (0, np.pi, 128), volume, spacing, det_count))


@parameterized(params)
def test_error(batch_size, angles, volume, spacing, det_count):
    if volume is None:
        volume = Volume2D()
        volume.set_size(det_count, det_count)

    radon = tr.ParallelBeam(det_count, angles, spacing, volume)

    f = random_symbolic_function(radon.volume.height, radon.volume.width)
    x = symbolic_discretize(f, radon.volume.height, radon.volume.width)

    f.scale(*radon.volume.voxel_size)
    f.move(*radon.volume.center)

    tx = torch.FloatTensor(x).unsqueeze(0).repeat(batch_size, 1, 1).to(device)

    y = symbolic_forward(f, radon.angles.cpu(), radon.projection.cfg).cpu().numpy()
    ty = radon.forward(tx)
    assert_equal(ty.size(0), batch_size)

    max_error = 2e-3 * (512 / y.shape[0]) * (512 / y.shape[1])

    description = f"Angles: {angles}\nVolume: {volume}\nSpacing: {spacing}, Count: {det_count}, Precision: float"
    test_helper.compare_images(y, ty, max_error, description)

    back_max_error = 1e-3
    test_helper.backward_check(tx, ty, radon, description, back_max_error)

    if batch_size % 4 == 0:
        ty = radon.forward(tx.half())

        description = f"Angles: {angles}\nVolume: {volume}\nSpacing: {spacing}, Count: {det_count}, Precision: half"
        test_helper.compare_images(y, ty, max_error, description)
