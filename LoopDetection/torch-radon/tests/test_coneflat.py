from torch_radon.volumes import Volume3D
from .utils import relative_error
import astra
from nose.tools import assert_less, assert_equal
import torch
import numpy as np
import torch_radon as tr
from parameterized import parameterized
import matplotlib.pyplot as plt

device = torch.device('cuda')

full_angles = np.linspace(0, 2*np.pi, 128).astype(np.float32)
many_angles = np.linspace(0, np.pi, 90).astype(np.float32)

params = []
for batch_size in [1, 8]:
    for volume_size in [64, 81]:
        for angles in [full_angles, many_angles]:
            for spacing in [1.0, 0.5, 1.3, 2.0]:
                for distances in [(1.5, 1.5), (2.0, 2.0), (1.2, 3.0)]:
                    for det_count in [1.0, 1.5]:
                        params.append((device, batch_size, volume_size, angles, spacing, distances, det_count))

half_params = [x for x in params if x[1] % 4 == 0]


@parameterized(params)
def test_fanflat_error(device, batch_size, volume_size, angles, det_spacing, distances, det_count):
    # generate random images
    det_count = int(det_count * volume_size)
    x = np.random.uniform(0.0, 1.0, (volume_size, volume_size, volume_size)).astype(np.float32)

    s_dist, d_dist = distances
    s_dist *= volume_size
    d_dist *= volume_size

    # astra
    vol_geom = astra.create_vol_geom(x.shape[1], x.shape[2], x.shape[0])
    proj_geom = astra.create_proj_geom('cone', det_spacing, det_spacing, det_count, det_count, angles, s_dist, d_dist)

    proj_id, astra_y = astra.create_sino3d_gpu(x, proj_geom, vol_geom)

    rec_id = astra.data3d.create('-vol', vol_geom)

    cfg = astra.astra_dict('BP3D_CUDA')
    cfg['ReconstructionDataId'] = rec_id
    cfg['ProjectionDataId'] = proj_id
    alg_id = astra.algorithm.create(cfg)
    astra.algorithm.run(alg_id, 1)

    astra_y = astra_y.transpose(1, 0, 2)
    astra_bp = astra.data3d.get(rec_id)

    # TODO clean astra structures

    # our implementation
    volume = Volume3D()
    volume.set_size(volume_size, volume_size, volume_size)
    radon = tr.ConeBeam(det_count, angles, s_dist, d_dist, det_spacing_u=det_spacing, volume=volume)
    x = torch.FloatTensor(x).view(1, x.shape[0], x.shape[1], x.shape[2]).repeat(batch_size, 1, 1, 1).to(device)

    our_fp = radon.forward(x)
    our_bp = radon.backward(our_fp)

    our_fp = our_fp.cpu().numpy()
    batch_error = max([relative_error(our_fp[0], our_fp[i]) for i in range(1, batch_size)] + [0])
    forward_error = relative_error(astra_y, our_fp[0])

    our_bp = our_bp.cpu().numpy()
    batch_error_back = max([relative_error(our_bp[0], our_bp[i]) for i in range(1, batch_size)] + [0])
    back_error = relative_error(astra_bp, our_bp[0])

    if not forward_error < 2e-2:
        fig, ax = plt.subplots(3, 3)
        ax = ax.ravel()
        ax[0].imshow(astra_y[0])
        ax[1].imshow(our_fp[0, 0])
        ax[2].imshow(np.abs(our_fp[0, 0] - astra_y[0]))
        ax[3].imshow(astra_y[len(angles)//2])
        ax[4].imshow(our_fp[0, len(angles)//2])
        ax[5].imshow(np.abs(our_fp[0, len(angles)//2] - astra_y[len(angles)//2]))
        ax[6].imshow(astra_y[-1])
        ax[7].imshow(our_fp[0, -1])
        ax[8].imshow(np.abs(our_fp[0, -1] - astra_y[-1]))
        plt.show()

    print(f"batch: {batch_size}, size: {volume_size}, angles: {len(angles)}, spacing: {det_spacing}, distances: {distances}, det_count:{det_count}, forward: {forward_error}, back: {back_error}")

    # TODO better checks
    assert_less(batch_error, 1e-6)
    assert_less(forward_error, 2e-2)
    assert_less(batch_error_back, 1e-6)
    assert_less(back_error, 3e-3)


@parameterized(half_params)
def test_half(device, batch_size, volume_size, angles, det_spacing, distances, det_count):
    # generate random images
    det_count = int(det_count * volume_size)
    x = np.random.uniform(0.0, 1.0, (batch_size, volume_size, volume_size, volume_size)).astype(np.float32)

    s_dist, d_dist = distances
    s_dist *= volume_size
    d_dist *= volume_size

    volume = Volume3D()
    volume.set_size(volume_size, volume_size, volume_size)
    radon = tr.ConeBeam(det_count, angles, s_dist, d_dist, det_spacing_u=det_spacing, volume=volume)
    x = torch.FloatTensor(x).to(device)

    single_fp = radon.forward(x) / len(angles)
    single_bp = radon.backward(single_fp)

    half_fp = radon.forward(x.half()) / len(angles)
    half_bp = radon.backward(half_fp)

    forward_error = relative_error(single_fp.cpu().numpy(), half_fp.float().cpu().numpy())
    back_error = relative_error(single_bp.cpu().numpy(), half_bp.float().cpu().numpy())

    print(f"batch: {batch_size}, size: {volume_size}, angles: {len(angles)}, spacing: {det_spacing}, distances: {distances}, det_count:{det_count}, forward: {forward_error}, back: {back_error}")

    # TODO better checks
    assert_less(forward_error, 3e-3)
    assert_less(back_error, 3e-3)
