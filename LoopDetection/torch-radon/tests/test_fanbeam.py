import numpy as np
import torch
from nose.tools import assert_equal
from parameterized import parameterized
import random

from .utils import random_symbolic_function, symbolic_discretize, symbolic_forward, TestHelper

from torch_radon.volumes import Volume2D
import torch_radon as tr

# tr.set_log_level(tr.DEBUG)

random.seed(42)
device = torch.device('cuda')
test_helper = TestHelper("fanbeam")

# (batch_size, angles, volume, spacing, det_count, src_dist, det_dist)
params = []

# check different batch sizes
for batch_size in [1, 3, 17, 32]:
    params.append((batch_size, (0, 2*np.pi, 128), None, 2.0, 128, 128, 128))

# check few and many angles which are not multiples of 16
for angles in [(0, 2*np.pi, 19), (0, 2*np.pi, 803)]:
    params.append((4, angles, None, 2.0, 128, 128, 128))

# change volume size
for height, width in [(128, 256), (256, 128), (75, 149), (81, 81)]:
    s = max(height, width)
    volume = Volume2D()
    volume.set_size(height, width)
    params.append((4, (0, 2*np.pi, 64), volume, 2.0, s, s, s))

# change volume scale and center
for center in [(0, 0), (17, -25), (53, 49)]:
    for voxel_size in [(1, 1), (0.75, 0.75), (1.5, 1.5), (0.7, 1.3), (1.3, 0.7)]:
        det_count = int(179 * max(voxel_size[0], 1) * max(voxel_size[1], 1) * np.sqrt(2))
        volume = Volume2D(center, voxel_size)
        volume.set_size(179, 123)
        params.append((4, (0, 2*np.pi, 128), volume, 2.0, det_count, det_count, det_count))

for spacing in [1.0, 0.5, 1.3, 2.0]:
    for det_count in [79, 128, 243]:
        for src_dist, det_dist in [(128, 128), (64, 128), (128, 64), (503, 503)]:
            volume = Volume2D()
            volume.set_size(128, 128)
            params.append((4, (0, 2*np.pi, 128), volume, spacing, det_count, src_dist, det_dist))

# params.append((4, (0, 6.283185307179586, 128), 128, 2.0, 243, 128, 64))


@parameterized(params)
def test_error(batch_size, angles, volume, spacing, det_count, src_dist, det_dist):
    if volume is None:
        volume = Volume2D()
        volume.set_size(det_count, det_count)
    radon = tr.FanBeam(det_count, angles, src_dist, det_dist, spacing, volume)

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


# full_angles = np.linspace(0, 2 * np.pi, 180).astype(np.float32)
# limited_angles = np.linspace(0.2 * np.pi, 0.5 * np.pi, 50).astype(np.float32)
# sparse_angles = np.linspace(0, 2 * np.pi, 60).astype(np.float32)
# many_angles = np.linspace(0, 2 * np.pi, 800).astype(np.float32)

# params = []
# for batch_size in [1, 8]:
#     for image_size in [128, 151]:
#         for angles in [full_angles, limited_angles, sparse_angles, many_angles]:
#             for spacing in [1.0, 0.5, 1.3, 2.0]:
#                 for distances in [(1.2, 1.2), (2.0, 2.0), (1.2, 3.0)]:
#                     for det_count in [1.0, 1.5]:
#                         params.append((device, batch_size, image_size, angles, spacing, distances, det_count))

# half_params = [x for x in params if x[1] % 4 == 0]


# @parameterized(params)
# def test_fanbeam_error(device, batch_size, image_size, angles, spacing, distances, det_count):
#     # generate random images
#     # generate random images
#     det_count = int(det_count * image_size)
#     x = generate_random_images(1, image_size)[0]

#     s_dist, d_dist = distances
#     s_dist *= image_size
#     d_dist *= image_size

#     # astra
#     vol_geom = astra.create_vol_geom(x.shape[0], x.shape[1])
#     proj_geom = astra.create_proj_geom('fanflat', spacing, det_count, angles, s_dist, d_dist)
#     proj_id = astra.create_projector('cuda', proj_geom, vol_geom)

#     id, astra_y = astra.create_sino(x, proj_id)
#     _, astra_bp = astra.create_backprojection(astra_y, proj_id)

#     # TODO clean astra structures

#     # our implementation
#     radon = tr.FanBeam(det_count=det_count, det_spacing=spacing, angles=angles,
#                        src_dist=s_dist, det_dist=d_dist, volume=image_size)
#     x = torch.FloatTensor(x).to(device).view(1, x.shape[0], x.shape[1])
#     # repeat data to fill batch size
#     x = torch.cat([x] * batch_size, dim=0)

#     our_fp = radon.forward(x)
#     our_bp = radon.backprojection(our_fp)

#     forward_error = relative_error(astra_y, our_fp[0].cpu().numpy())
#     back_error = relative_error(astra_bp, our_bp[0].cpu().numpy())

#     # if back_error > 5e-3:
#     #     plt.imshow(astra_bp)
#     #     plt.figure()
#     #     plt.imshow(our_bp[0].cpu().numpy())
#     #     plt.show()
#     print(np.max(our_fp.cpu().numpy()), np.max(our_bp.cpu().numpy()))

#     print(
#         f"batch: {batch_size}, size: {image_size}, angles: {len(angles)}, spacing: {spacing}, distances: {distances}, forward: {forward_error}, back: {back_error}")
#     # TODO better checks
#     assert_less(forward_error, 1e-2)
#     assert_less(back_error, 5e-3)


# @parameterized(half_params)
# def test_half(device, batch_size, image_size, angles, spacing, distances, det_count):
#     # generate random images
#     det_count = int(det_count * image_size)
#     x = generate_random_images(batch_size, image_size)

#     s_dist, d_dist = distances
#     s_dist *= image_size
#     d_dist *= image_size

#     # our implementation
#     radon = tr.FanBeam(det_count=det_count, det_spacing=spacing, angles=angles,
#                        src_dist=s_dist, det_dist=d_dist, volume=image_size)
#     x = torch.FloatTensor(x).to(device)

#     # divide by len(angles) to avoid half-precision overflow
#     sinogram = radon.forward(x) / len(angles)
#     single_precision = radon.backprojection(sinogram)

#     h_sino = radon.forward(x.half()) / len(angles)
#     half_precision = radon.backprojection(h_sino)
#     print(torch.min(half_precision).item(), torch.max(half_precision).item())

#     forward_error = relative_error(sinogram.cpu().numpy(), h_sino.cpu().numpy())
#     back_error = relative_error(single_precision.cpu().numpy(), half_precision.cpu().numpy())

#     print(
#         f"batch: {batch_size}, size: {image_size}, angles: {len(angles)}, spacing: {spacing}, forward: {forward_error}, back: {back_error}")

#     assert_less(forward_error, 1e-3)
#     assert_less(back_error, 1e-3)
