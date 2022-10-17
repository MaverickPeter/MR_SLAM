#ifndef RMATH_SIMPLE_H
#define RMATH_SIMPLE_H

#include <stdint.h>
#include <limits>

#define EXPECT_FALSE(x) __builtin_expect(x, false)
#define EXPECT_TRUE(x) __builtin_expect(x, true)

#pragma GCC push_options
#pragma GCC optimize("03", "no-fast-math")

namespace rosh
{
    namespace internals
    {
        namespace exp
        {
            constexpr int N = 32;
            constexpr double threshold = 88.7228240;
            constexpr double inv_ln2_n = 0x1.71547652b82fep+0 * N;
            constexpr double shift = 0x1.8p+52;

            constexpr uint64_t table[32] = {
                0x3ff0000000000000,
                0x3fefd9b0d3158574,
                0x3fefb5586cf9890f,
                0x3fef9301d0125b51,
                0x3fef72b83c7d517b,
                0x3fef54873168b9aa,
                0x3fef387a6e756238,
                0x3fef1e9df51fdee1,
                0x3fef06fe0a31b715,
                0x3feef1a7373aa9cb,
                0x3feedea64c123422,
                0x3feece086061892d,
                0x3feebfdad5362a27,
                0x3feeb42b569d4f82,
                0x3feeab07dd485429,
                0x3feea47eb03a5585,
                0x3feea09e667f3bcd,
                0x3fee9f75e8ec5f74,
                0x3feea11473eb0187,
                0x3feea589994cce13,
                0x3feeace5422aa0db,
                0x3feeb737b0cdc5e5,
                0x3feec49182a3f090,
                0x3feed503b23e255d,
                0x3feee89f995ad3ad,
                0x3feeff76f2fb5e47,
                0x3fef199bdd85529c,
                0x3fef3720dcef9069,
                0x3fef5818dcfba487,
                0x3fef7c97337b9b5f,
                0x3fefa4afa2a490da,
                0x3fefd0765b6e4540,
            };

            constexpr double poly[4] = {
                1.6938658739001192597e-6l,
                2.3459849135121584372e-4l,
                2.1660849391256500861e-2l,
                9.9999999992833901766e-1l,
            };

        }

        namespace log
        {
            struct LogTableEntry
            {
                double invc, logc;
            };

            constexpr uint32_t off = 0x3f330000;
            constexpr int N = 16;
            constexpr double ln2 = 0x1.62e42fefa39efp-1;
            constexpr double poly[3] = {
                -0x1.00ea348b88334p-2,
                0x1.5575b0be00b6ap-2,
                -0x1.ffffef20a4123p-2,
            };
            constexpr LogTableEntry table[16] = {
                {0x1.661ec79f8f3bep+0, -0x1.57bf7808caadep-2},
                {0x1.571ed4aaf883dp+0, -0x1.2bef0a7c06ddbp-2},
                {0x1.49539f0f010bp+0, -0x1.01eae7f513a67p-2},
                {0x1.3c995b0b80385p+0, -0x1.b31d8a68224e9p-3},
                {0x1.30d190c8864a5p+0, -0x1.6574f0ac07758p-3},
                {0x1.25e227b0b8eap+0, -0x1.1aa2bc79c81p-3},
                {0x1.1bb4a4a1a343fp+0, -0x1.a4e76ce8c0e5ep-4},
                {0x1.12358f08ae5bap+0, -0x1.1973c5a611cccp-4},
                {0x1.0953f419900a7p+0, -0x1.252f438e10c1ep-5},
                {0x1p+0, 0x0p+0},
                {0x1.e608cfd9a47acp-1, 0x1.aa5aa5df25984p-5},
                {0x1.ca4b31f026aap-1, 0x1.c5e53aa362eb4p-4},
                {0x1.b2036576afce6p-1, 0x1.526e57720db08p-3},
                {0x1.9c2d163a1aa2dp-1, 0x1.bc2860d22477p-3},
                {0x1.886e6037841edp-1, 0x1.1058bc8a07ee1p-2},
                {0x1.767dcf5534862p-1, 0x1.4043057b6ee09p-2},
            };

        }

        inline float sin_poly(float x)
        {
            constexpr float poly[6] = {
                -2.0366233e-8f,
                2.6998228e-6f,
                -1.980874e-4f,
                8.3324076e-3f,
                -1.6666553e-1f,
                9.999996e-1f,
            };

            const float xx = x * x;
            float a = poly[0] * xx + poly[1];
            float b = poly[2] * xx + poly[3];
            float c = poly[4] * xx + poly[5];
            const float xx2 = xx * xx;
            float ab = (a * xx2 + b);
            return (ab * xx2 + c) * x;
        }

        inline float cos_poly(float x)
        {
            const float xx = x * x;
            float u = -2.1978884e-7f;
            u = u * xx + 2.4204402e-5f;
            u = u * xx + -1.3858916e-3f;
            u = u * xx + 4.1659822e-2f;
            u = u * xx + -4.9999427e-1f;
            return u * xx + 9.9999922e-1f;
        }

        inline float erf_poly(float x)
        {
            constexpr float poly[6] = {
                0.0000430638f,
                0.0002765672f,
                0.0001520143f,
                0.0092705272f,
                0.0422820123f,
                0.0705230784f,
            };

            float a = poly[0] * x + poly[1];
            float b = poly[2] * x + poly[3];
            float c = poly[4] * x + poly[5];
            float x2 = x * x;
            float x3 = x2 * x;
            float ab = (a * x2 + b);
            return ab * x3 + (c * x + 1.0f);
        }

        inline uint32_t as_uint(float f)
        {
            union
            {
                float f;
                uint32_t i;
            } u = {f};
            return u.i;
        }

        inline float as_float(uint32_t i)
        {
            union
            {
                uint32_t i;
                float f;
            } u = {i};
            return u.f;
        }

        inline uint64_t as_uint64(double f)
        {
            union
            {
                double f;
                uint64_t i;
            } u = {f};
            return u.i;
        }

        inline double as_double(uint64_t i)
        {
            union
            {
                uint64_t i;
                double f;
            } u = {i};
            return u.f;
        }

    }

    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float pi = 3.14159265358979f;

    inline float sqrt(float x)
    {
        __asm__("sqrtss %1, %0"
                : "=x"(x)
                : "x"(x));
        return x;
    }

    inline float hypot(float x, float y)
    {
        // not safe, may overflow
        return sqrt(x * x + y * y);
    }

    inline float sq(float x)
    {
        return x * x;
    }

    inline float abs(float x)
    {
        const uint32_t t = internals::as_uint(x) & 0x7fffffff;
        return internals::as_float(t);
    }

    // computes abs(x)*sign(y)
    inline float copy_sign(float x, float y)
    {
        const uint32_t sign = internals::as_uint(y) & 0x80000000;
        const uint32_t t = internals::as_uint(x) | sign;
        return internals::as_float(t);
    }

    inline float floor(float x)
    {
        return int(x) - (x < 0);
    }

    inline float max(float x, float y)
    {
        return x > y ? x : y;
    }

    inline float min(float x, float y)
    {
        return x > y ? y : x;
    }

    inline float exp(float x)
    {
        using namespace internals::exp;

        // if |x| is too large return 0 or +inf
        if (EXPECT_FALSE(x < -threshold))
            return 0.0f;

        if (EXPECT_FALSE(x > threshold))
            return std::numeric_limits<float>::infinity();

        // use double precision
        const double xd = double(x);

        // (x * n)/log(2) = k + r   where k is an integer and r is in [-1/2, 1/2]
        double z = xd * inv_ln2_n;
        double kd = z + shift;
        uint64_t ki = internals::as_uint64(kd);
        kd -= shift;
        double r = z - kd;

        // x = log(2) * (k/n + r/n)
        // exp(x) = 2^(floor(k/n) + (k%n)/n + r/n)
        // 2^((k%n)/n) is looked up in the table
        uint64_t t = table[ki % N];

        // 2^floor(k/n) is a shift of -5 to divide by n and a shift of 52 to put floor(k/n) into the exponent bits
        // summing on the uint representation is used to compute the product because the mantissa of 2^floor(k/n) is zero
        t += ki << (52 - 5);

        double s = internals::as_double(t);

        // 2^(r/n) is approximated using a polynomial
        z = poly[0] * r + poly[1];
        double r2 = r * r;
        double y = poly[2] * r + poly[3];
        y = z * r2 + y;

        return float(y * s);
    }

    inline float log(float x)
    {
        using namespace internals::log;

        uint32_t ix = internals::as_uint(x);

        if (EXPECT_FALSE(ix - 0x00800000 >= 0x7f800000 - 0x00800000))
        {
            /* x < 0x1p-126 or inf or nan.  */
            if (ix * 2 == 0)
                return -rosh::inf;
            if (ix == 0x7f800000) /* log(inf) == inf.  */
                return x;
            if ((ix & 0x80000000) || ix * 2 >= 0xff000000)
                return rosh::nan;

            /* x is subnormal, normalize it.  */
            ix = internals::as_uint(x * 0x1p23f);
            ix -= 23 << 23;
        }

        uint32_t tmp = ix - off;

        int i = (tmp >> 19) % N;
        int k = (int32_t)tmp >> 23;
        uint32_t iz = ix - (tmp & uint32_t(0x1ff) << 23);

        double invc = table[i].invc;
        double logc = table[i].logc;

        double z = double(internals::as_float(iz));
        // x = z * 2^k
        // x = (z / c) * c * 2^k
        // log(x) = log1p(z/c-1) + log(c) + k*Ln2
        double r = z * invc - 1;
        double y0 = logc + double(k) * ln2;
        /* Pipelined polynomial evaluation to approximate log1p(r).  */
        double r2 = r * r;
        double y = poly[1] * r + poly[2];
        y = poly[0] * r2 + y;
        y = y * r2 + (y0 + r);

        return (float)y;
    }

    inline float sin(float x)
    {
        constexpr float twopi = 6.283185307179586f;
        constexpr float invtwopi = 0.15915494309189535f;

        // x is now in range [-pi, pi]
        x -= rosh::floor(x * invtwopi + 0.5f) * twopi;

        return internals::sin_poly(x);
    }

    inline float cos(float x)
    {
        constexpr float twopi = 6.283185307179586f;
        constexpr float invtwopi = 0.15915494309189535f;

        // x is now in range [-pi, pi]
        x -= rosh::floor(x * invtwopi + 0.5f) * twopi;

        return internals::cos_poly(x);
    }

    // specify sorted = true when you are processing "long" sequences of positive/negative values.
    // long sequences with same sign ===> few branch mispredictions ===> "if" is more perfomant
    template <bool sorted = false>
    float erf(float x)
    {
        if (sorted)
        {
            if (x > 0)
            {
                float y = internals::erf_poly(x);
                y = y * y;
                y = y * y;
                y = y * y;
                return 1.0f - 1.0f / (y * y);
            }
            else
            {
                float y = internals::erf_poly(-x);
                y = y * y;
                y = y * y;
                y = y * y;
                return 1.0f / (y * y) - 1.0f;
            }
        }
        else
        {
            float y = internals::erf_poly(rosh::abs(x));
            y = y * y;
            y = y * y;
            y = y * y;
            y = 1.0f - 1.0f / (y * y);

            return copy_sign(y, x);
        }
    }
}

#pragma GCC pop_options
#endif