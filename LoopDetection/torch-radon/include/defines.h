#ifndef TORCH_RADON_DEFINES_H
#define TORCH_RADON_DEFINES_H

typedef unsigned int uint;

#define PRECISION_FLOAT 1
#define PRECISION_HALF 0

#define PARALLEL 0
#define FANBEAM 1
#define CONEFLAT 2 // Cone beam geometry with circular/helical source curve and flat detector

#define TEX_1D_LAYERED 0
#define TEX_2D_LAYERED 1
#define TEX_3D 2

#endif //TORCH_RADON_DEFINES_H
