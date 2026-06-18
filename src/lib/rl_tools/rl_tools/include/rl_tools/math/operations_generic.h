#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_GENERIC_MATH_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_GENERIC_MATH_H

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

#ifdef _MSC_VER
#include <limits>
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::math {

    template<typename T>
    constexpr T PI = 3.141592653589793238462643383279502884L;

    template<typename T>
    constexpr T FRAC_2_SQRTPI = 1.128379167095512573896158903121545172L;
    template<typename T>
    constexpr T SQRT1_2 = 0.707106781186547524400844362104849039L;

    // Implementing sqrt using the Babylonian method (also known as Heron's method)
    template<typename T>
    T sqrt(const devices::math::Generic&, T x) {
        if (x < 0) return -1; // Return -1 for negative numbers to indicate an error
        T guess = x / 2.0;
        T epsilon = 0.00001;
        while ((guess * guess - x) > epsilon || (x - guess * guess) > epsilon) {
            guess = (guess + x / guess) / 2.0;
        }
        return guess;
    }

    template<typename T>
    T exp(const devices::math::Generic&, T x) {
        T sum = 1.0;
        T term = 1.0;
        for (int i = 1; i < 20; ++i) {
            term *= x / i;
            sum += term;
        }
        return sum;
    }

    template<typename T>
    T tanh(const devices::math::Generic&, T x) {
        if (x > 20.0) return 1.0;
        if (x < -20.0) return -1.0;
        T ex = exp(devices::math::Generic(), x);
        T emx = exp(devices::math::Generic(), -x);
        return (ex - emx) / (ex + emx);
    }


    template<typename T>
    T sin(const devices::math::Generic&, T x) {
        T term = x;
        T sum = x;
        for (int i = 1; i < 10; ++i) {
            term *= -x * x / (2 * i * (2 * i + 1));
            sum += term;
        }
        return sum;
    }

    template<typename T>
    T cos(const devices::math::Generic&, T x) {
        T term = 1.0;
        T sum = 1.0;
        for (int i = 1; i < 10; ++i) {
            term *= -x * x / (2 * i * (2 * i - 1));
            sum += term;
        }
        return sum;
    }

    // based on https://git.musl-libc.org/cgit/musl/tree/src/math/acosf.c?h=v1.2.5
    /* origin: FreeBSD /usr/src/lib/msun/src/e_acosf.c */
    /*
     * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
     */
    /*
     * ====================================================
     * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
     *
     * Developed at SunPro, a Sun Microsystems, Inc. business.
     * Permission to use, copy, modify, and distribute this
     * software is freely granted, provided that this notice
     * is preserved.
     * ====================================================
     */

    namespace generic::acos{
        static constexpr float
        pio2_hi = 1.5707962513e+00f, /* 0x3fc90fda */
        pio2_lo = 7.5497894159e-08f, /* 0x33a22168 */
        pS0 =  1.6666586697e-01f,
        pS1 = -4.2743422091e-02f,
        pS2 = -8.6563630030e-03f,
        qS1 = -7.0662963390e-01f;
        static inline float R(float z){
            float p, q;
            p = z*(pS0+z*(pS1+z*pS2));
            q = 1.0f+z*qS1;
            return p/q;
        }
        inline uint32_t get_float_bits(float f) {
            uint32_t bits = 0;
            unsigned char* f_bytes = reinterpret_cast<unsigned char*>(&f);
            unsigned char* bits_bytes = reinterpret_cast<unsigned char*>(&bits);

            for (uint32_t i = 0; i < sizeof(float); ++i) {
                bits_bytes[i] = f_bytes[i];  // Copy byte by byte
            }

            return bits;
        }
        inline float set_float_bits(uint32_t bits) {
            float f = 0.0f;
            unsigned char* f_bytes = reinterpret_cast<unsigned char*>(&f);
            unsigned char* bits_bytes = reinterpret_cast<unsigned char*>(&bits);

            for (uint32_t i = 0; i < sizeof(float); ++i) {
                f_bytes[i] = bits_bytes[i];  // Copy byte by byte
            }

            return f;
        }
    }
    template<typename T>
    T acos(const devices::math::Generic& dev, T x) {
        float z,w,s,c,df;
        uint32_t hx,ix;

        hx = generic::acos::get_float_bits(x);
        ix = hx & 0x7fffffff;
        /* |x| >= 1 or nan */
        if (ix >= 0x3f800000) {
            if (ix == 0x3f800000) {
                if (hx >> 31)
                    return 2*generic::acos::pio2_hi + 0x1p-120f;
                return 0;
            }
            return 0/(x-x);
        }
        /* |x| < 0.5 */
        if (ix < 0x3f000000) {
            if (ix <= 0x32800000) /* |x| < 2**-26 */
                return generic::acos::pio2_hi + 0x1p-120f;
            return generic::acos::pio2_hi - (x - (generic::acos::pio2_lo-x*generic::acos::R(x*x)));
        }
        /* x < -0.5 */
        if (hx >> 31) {
            z = (1+x)*0.5f;
            s = math::sqrt(dev, z);
            w = generic::acos::R(z)*s-generic::acos::pio2_lo;
            return 2*(generic::acos::pio2_hi - (s+w));
        }
        /* x > 0.5 */
        z = (1-x)*0.5f;
        s = math::sqrt(dev, z);
        hx = generic::acos::get_float_bits(s);
        df = generic::acos::set_float_bits(hx&0xfffff000);
        c = (z-df*df)/(s+df);
        w = generic::acos::R(z)*s+c;
        return 2*(df+w);
    }

    template<typename TX, typename TY>
    RL_TOOLS_FUNCTION_PLACEMENT auto pow(const devices::math::Generic&, const TX x, const TY y) {
        if (y == 0) return static_cast<TX>(1);
        if (y < 0) return static_cast<TX>(1) / pow(devices::math::Generic(), x, -y);
        TX temp = pow(devices::math::Generic(), x, y / 2);
        if (static_cast<int>(y) % 2 == 0)
            return temp * temp;
        else
            return x * temp * temp;
    }

    template<typename T>
    T log(const devices::math::Generic&, T x) {
        if (x <= 0) return -1; // log is only defined for positive numbers
        T sum = 0.0;
        T term = (x - 1) / (x + 1);
        T term_squared = term * term;
        for (int i = 0; i < 10; ++i) {
            sum += term / (2 * i + 1);
            term *= term_squared;
        }
        return 2 * sum;
    }

    template<typename T>
    T floor(const devices::math::Generic&, const T x) {
        return static_cast<int>(x);
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(const devices::math::Generic&, const T x) {
        return x != x;
    }

    // template<typename T>
    // bool is_finite(const devices::math::Generic&, const T x) {
    //     if constexpr (std::is_same_v<T, float>) {
    //         return x <= 3.402823e+38f && x >= -3.402823e+38f;
    //     } else if constexpr (std::is_same_v<T, double>) {
    //         return x <= 1.7976931348623158e+308 && x >= -1.7976931348623158e+308;
    //     } else {
    //         return true; // Assume finite for other types
    //     }
    // }

    template<typename T>
    T clamp(const devices::math::Generic&, T x, T min, T max) {
        return x < min ? min : (x > max ? max : x);
    }

    template<typename T>
    T min(const devices::math::Generic&, T x, T y) {
        return x < y ? x : y;
    }

    template<typename T>
    T max(const devices::math::Generic&, T x, T y) {
        return x > y ? x : y;
    }

    template<typename T>
    T abs(const devices::math::Generic&, T x) {
        return x > 0 ? x : -x;
    }

    template<typename T>
    T nan(const devices::math::Generic&) {
#ifndef _MSC_VER
        return 0.0 / 0.0; // Produces NaN
#else
        return std::numeric_limits<T>::quiet_NaN();
#endif
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T fast_tanh(const devices::math::Generic& dev, T x){
        x = clamp(dev, x, static_cast<T>(-3.0), static_cast<T>(3.0));
        T x_squared = x * x;
        return x * (27 + x_squared) / (27 + 9 * x_squared);
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T fast_sigmoid(const devices::math::Generic& dev, T x){
        return (T)0.5 * fast_tanh(dev, (T)0.5 * x) + (T)0.5;
    }


    template<typename T>
    T atan(const devices::math::Generic&, T x) {
        // Constants for polynomial approximation
        T a = 0.9998660;
        T b = -0.3302995;
        T c = 0.1801410;
        T d = -0.0851330;
        T e = 0.0208351;

        // Polynomial approximation for atan(x) in the range [-1, 1]
        T abs_x = x < 0 ? -x : x;
        T x2 = x * x;
        T result = ((a * x2 + b) * x2 + c) * x2 + d;
        result = result * x2 + e;
        result = result * abs_x;

        // Adjust for input sign
        result = x < 0 ? -result : result;

        return result;
    }

    template<typename T>
    T atan2(const devices::math::Generic&, T y, T x) {
        if (x > 0) {
            return atan(y / x);
        } else if (x < 0 && y >= 0) {
            return atan(y / x) + PI<T>;
        } else if (x < 0 && y < 0) {
            return atan(y / x) - PI<T>;
        } else if (x == 0 && y > 0) {
            return PI<T> / 2;
        } else if (x == 0 && y < 0) {
            return -PI<T> / 2;
        }
        return 0;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
