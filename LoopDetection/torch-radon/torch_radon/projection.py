from .cuda_backend import ProjectionCfg

class Projection:
    PARALLEL = 0
    FANBEAM = 1
    CONE_FLAT = 2

    def __init__(self, cfg: ProjectionCfg):
        self.cfg = cfg

    @staticmethod
    def parallel_beam(det_count, det_spacing=1.0):
        return Projection(ProjectionCfg(det_count, det_spacing))

    @staticmethod
    def fanbeam(src_dist, det_dist, det_count, det_spacing=1.0):
        return Projection(ProjectionCfg(
            det_count, det_spacing,
            0, 1.0,
            src_dist, det_dist,
            0.0, 0.0,
            Projection.FANBEAM
        ))

    @staticmethod
    def coneflat(src_dist, det_dist, det_count_u, det_spacing_u=1.0, det_count_v=-1, det_spacing_v=-1.0, pitch=0.0, base_z=0.0):
        return Projection(ProjectionCfg(
            det_count_u, det_spacing_u,
            det_count_v, det_spacing_v,
            src_dist, det_dist,
            pitch, base_z,
            Projection.CONE_FLAT
        ))

    def is_2d(self):
        return self.cfg.is_2d()
