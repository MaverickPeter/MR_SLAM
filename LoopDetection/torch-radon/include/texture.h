#ifndef TORCH_RADON_TEXTURE_CACHE_H
#define TORCH_RADON_TEXTURE_CACHE_H

#include <cuda.h>
#include <cuda_runtime.h>
#include "utils.h"
#include "cache.h"
#include "defines.h"


class TextureConfig {
public:
    int device;

    int depth;
    int height;
    int width;

    bool is_layered;

    int channels;
    int precision;

    TextureConfig(int dv, int _z, int _y, int _x, bool layered, int c, int p);

    bool operator==(const TextureConfig &o) const;

    int get_texture_type() const;
};

TextureConfig create_1Dlayered_texture_config(int device, int size, int layers, int channels, int precision);

std::ostream &operator<<(std::ostream &os, TextureConfig const &m);

class Texture {
    cudaArray *array = nullptr;
    TextureConfig cfg;

public:
    cudaSurfaceObject_t surface = 0;
    cudaTextureObject_t texture = 0;

    Texture(TextureConfig c);
    void put(const float *data);
    void put(const unsigned short *data);

    bool matches(TextureConfig& k);

    ~Texture();
};


typedef Cache<TextureConfig, Texture> TextureCache;

#endif
