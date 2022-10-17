#include "texture.h"
#include "parameter_classes.h"

template< typename T> void radon_backward_cuda(
        const T *x, const float *angles, T *y, TextureCache &tex_cache,
        const VolumeCfg& vol_cfg, const ProjectionCfg& proj_cfg, const ExecCfg& exec_cfg,
        const int batch_size, const int device
);

template< typename T> void radon_backward_cuda_3d(
        const T *x, const float *angles, T *y, TextureCache &tex_cache,
        const VolumeCfg& vol_cfg, const ProjectionCfg& proj_cfg, const ExecCfg& exec_cfg,
        const int batch_size, const int device
);