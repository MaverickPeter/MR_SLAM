#include <vector>

#include "parameter_classes.h"

struct Gaussian
{
    float k;
    float cx;
    float cy;
    float a, b;

    Gaussian(float _k, float _cx, float _cy, float _a, float _b);

    float line_integral(float s_x, float s_y, float e_x, float e_y) const;
    float evaluate(float x, float y) const;

    void move(float dx, float dy);
    void scale(float sx, float sy);
};

struct Ellipse
{
    float k;
    float cx;
    float cy;

    float radius_x;
    float radius_y;
    float aspect;

    Ellipse(float _k, float _cx, float _cy, float rx, float ry);

    float line_integral(float s_x, float s_y, float e_x, float e_y) const;
    float evaluate(float x, float y) const;

    void move(float dx, float dy);
    void scale(float sx, float sy);
};

class SymbolicFunction
{
    std::vector<Gaussian> gaussians;
    std::vector<Ellipse> ellipses;

    float min_x;
    float min_y;
    float max_x;
    float max_y;

public:
    SymbolicFunction(float h, float w);

    void add_gaussian(float k, float cx, float cy, float a, float b);
    void add_ellipse(float k, float cx, float cy, float r, float a);

    void move(float dx, float dy);
    void scale(float sx, float sy);


    float max_distance_from_origin() const;

    void discretize(float *data, int h, int w) const;
    float line_integral(float s_x, float s_y, float e_x, float e_y) const;
};

void symbolic_forward(const SymbolicFunction &f, const ProjectionCfg &proj, const float *angles, const int n_angles, float *sinogram);