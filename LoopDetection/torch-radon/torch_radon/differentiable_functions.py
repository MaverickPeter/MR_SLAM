import torch
from torch.autograd import Function

from . import cuda_backend


class RadonForward(Function):
    @staticmethod
    def forward(ctx, x, angles, tex_cache, vol_cfg, proj_cfg, exec_cfg_generator, exec_cfg=None):
        exec_cfg = exec_cfg_generator(vol_cfg, proj_cfg,  x.dtype == torch.half) if exec_cfg is None else exec_cfg
        sinogram = cuda_backend.forward(x, angles, tex_cache, vol_cfg, proj_cfg, exec_cfg)
        ctx.tex_cache = tex_cache
        ctx.vol_cfg = vol_cfg
        ctx.proj_cfg = proj_cfg.copy()
        ctx.exec_cfg_generator = exec_cfg_generator
        ctx.save_for_backward(angles)

        return sinogram

    @staticmethod
    def backward(ctx, grad_x):
        angles, = ctx.saved_tensors
        exec_cfg = ctx.exec_cfg_generator(ctx.vol_cfg, ctx.proj_cfg, grad_x.dtype == torch.half)
        grad = cuda_backend.backward(grad_x, angles, ctx.tex_cache, ctx.vol_cfg, ctx.proj_cfg, exec_cfg)
        return grad, None, None, None, None, None, None


class RadonBackprojection(Function):
    @staticmethod
    def forward(ctx, x, angles, tex_cache, vol_cfg, proj_cfg, exec_cfg_generator, exec_cfg=None):
        exec_cfg = exec_cfg_generator(vol_cfg, proj_cfg,  x.dtype == torch.half) if exec_cfg is None else exec_cfg
        image = cuda_backend.backward(x, angles, tex_cache,  vol_cfg, proj_cfg, exec_cfg)
        ctx.tex_cache = tex_cache
        ctx.vol_cfg = vol_cfg
        ctx.proj_cfg = proj_cfg.copy()
        ctx.exec_cfg_generator = exec_cfg_generator
        ctx.save_for_backward(angles)

        return image

    @staticmethod
    def backward(ctx, grad_x):
        angles, = ctx.saved_tensors
        exec_cfg = ctx.exec_cfg_generator(ctx.vol_cfg, ctx.proj_cfg, grad_x.dtype == torch.half)
        grad = cuda_backend.forward(grad_x, angles, ctx.tex_cache, ctx.vol_cfg, ctx.proj_cfg, exec_cfg)
        return grad, None, None, None, None, None, None
