import astra
import numpy as np


class AstraWrapper:
    def __init__(self, angles):
        self.angles = angles

        self.projectors = []
        self.algorithms = []
        self.data2d = []
        self.data3d = []

    def forward(self, x, spacing=1.0, det_count=-1):
        if det_count < 0:
            det_count = x.shape[1]

        vol_geom = astra.create_vol_geom(x.shape[1], x.shape[2], x.shape[0])
        phantom_id = astra.data3d.create('-vol', vol_geom, data=x)
        proj_geom = astra.create_proj_geom('parallel3d', spacing, 1.0, x.shape[0], det_count, -self.angles)

        proj_id, y = astra.creators.create_sino3d_gpu(phantom_id, proj_geom, vol_geom)

        self.projectors.append(proj_id)
        self.data3d.append(phantom_id)

        return proj_id, y
    # def forward(self, x):
    #     vol_geom = astra.create_vol_geom(x.shape[1], x.shape[2])
    #     proj_geom = astra.create_proj_geom('parallel', 1.0, x.shape[1], self.angles)
    #     proj_id = astra.create_projector('cuda', proj_geom, vol_geom)
    #
    #     self.projectors.append(proj_id)
    #
    #     ys = []
    #     for i in range(x.shape[0]):
    #         _, y = astra.create_sino(x[i], proj_id)
    #         ys.append(y.reshape(1, y.shape[0], y.shape[1]))
    #
    #     y = np.vstack(ys)
    #     return y

    def backproject(self, proj_id, s, bs):
        vol_geom = astra.create_vol_geom(s, s, bs)
        rec_id = astra.data3d.create('-vol', vol_geom)

        # Set up the parameters for a reconstruction algorithm using the GPU
        cfg = astra.astra_dict('BP3D_CUDA')
        cfg['ReconstructionDataId'] = rec_id
        cfg['ProjectionDataId'] = proj_id

        # Create the algorithm object from the configuration structure
        alg_id = astra.algorithm.create(cfg)
        astra.algorithm.run(alg_id, 1)

        self.algorithms.append(alg_id)
        self.data3d.append(rec_id)

        return astra.data3d.get(rec_id)

    def forward_single(self, x):
        vol_geom = astra.create_vol_geom(x.shape[0], x.shape[1])
        proj_geom = astra.create_proj_geom('parallel', 1.0, x.shape[0], self.angles)
        proj_id = astra.create_projector('cuda', proj_geom, vol_geom)

        self.projectors.append(proj_id)

        return astra.create_sino(x, proj_id)

    def fbp(self, x):
        s = x.shape[0]
        proj_id, _ = self.forward_single(x)
        vol_geom = astra.create_vol_geom(s, s)
        rec_id = astra.data2d.create('-vol', vol_geom)

        # create configuration
        cfg = astra.astra_dict('FBP_CUDA')
        cfg['ReconstructionDataId'] = rec_id
        cfg['ProjectionDataId'] = proj_id
        cfg['option'] = {'FilterType': 'Ram-Lak'}

        alg_id = astra.algorithm.create(cfg)
        astra.algorithm.run(alg_id)

        self.projectors.append(proj_id)
        self.algorithms.append(alg_id)
        self.data2d.append(rec_id)

        return astra.data2d.get(rec_id)

    def clean(self):
        # clean all astra stuff
        for pid in self.projectors:
            astra.projector.delete(pid)

        for pid in self.algorithms:
            astra.algorithm.delete(pid)

        for pid in self.data2d:
            astra.data2d.delete(pid)

        for pid in self.data3d:
            astra.data3d.delete(pid)

    def __del__(self):
        self.clean()