from turtle import width
from .cuda_backend import VolumeCfg


class Volume2D:
    def __init__(self, center=(0.0, 0.0), voxel_size=(1.0, 1.0)):
        self.height = -1
        self.width = -1

        self.center = center
        self.voxel_size = voxel_size

    def to_cfg(self) -> VolumeCfg:
        assert self.has_size()

        return VolumeCfg(
            0, self.height, self.width,
            0.0, self.center[1], self.center[0],
            1.0, self.voxel_size[1], self.voxel_size[0],
            False
        )

    def num_dimensions(self):
        return 2

    def has_size(self):
        return self.height > 0 and self.width > 0

    def set_size(self, height, width):
        self.height = height
        self.width = width

    def __str__(self) -> str:
        return f"Volume2D(height={self.height}, width={self.width}, center={self.center}, voxel_size={self.voxel_size})"


class Volume3D:
    def __init__(self, center=(0.0, 0.0, 0.0), voxel_size=(1.0, 1.0, 1.0)):
        self.depth = -1
        self.height = -1
        self.width = -1

        self.center = center
        self.voxel_size = voxel_size

    def has_size(self):
        return self.depth > 0 and self.height > 0 and self.width > 0

    def set_size(self, depth, height, width):
        self.depth = depth
        self.height = height
        self.width = width

    def shape(self):
        return (self.depth, self.height, self.width)

    def min(self):
        dx, dy, dz = self.center
        sx, sy, sz = self.voxel_size
        return [-self.width*sx / 2 + dx, -self.height*sy/2 + dy, -self.depth*sz/2 + dz]

    def max(self):
        dx, dy, dz = self.center
        sx, sy, sz = self.voxel_size
        return [self.width*sx / 2 + dx, self.height*sy/2 + dy, self.depth*sz/2 + dz]

    def to_cfg(self) -> VolumeCfg:
        assert self.has_size()

        return VolumeCfg(
            self.depth, self.height, self.width,
            self.center[2], self.center[1], self.center[0],
            self.voxel_size[2], self.voxel_size[1], self.voxel_size[0],
            True
        )

    def num_dimensions(self):
        return 3
