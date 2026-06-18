#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_MATH_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_MATH_OPERATIONS_CUDA_H

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

#include "operations_generic.h"

#include "../devices/cuda.h"
#include <limits>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::math {
    namespace cuda {
        template<typename T>
        constexpr bool check = utils::typing::is_same_v<T, __nv_bfloat16> || utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>;
    }

    // CUDA std
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T sqrt(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::sqrtf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::sqrt(x);
            } else {
                return ::hsqrt(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T tanh(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::tanhf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::tanh(x);
            } else {
                return (T)::tanhf((float)x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T exp(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::expf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::exp(x);
            } else {
                return ::hexp(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T sin(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::sinf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::sin(x);
            } else {
                return ::hsin(x);
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T asin(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::asinf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::asin(x);
            } else {
                return (T)::asinf((float)x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T cos(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::cosf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::cos(x);
            } else {
                return ::hcos(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T acos(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::acosf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::acos(x);
            } else {
                return (T)::acosf((float)x);
            }
        }
    }

    template<typename TX, typename TY>
    RL_TOOLS_FUNCTION_PLACEMENT auto pow(const devices::math::CUDA &, const TX x, const TY y) {
        static_assert(cuda::check<TX>, "CUDA math only supports float and double");
        static_assert(cuda::check<TY>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<TX, float> && utils::typing::is_same_v<TY, float>) {
            return ::powf(x, y);
        } else {
            if constexpr (utils::typing::is_same_v<TX, double> || utils::typing::is_same_v<TY, double>) {
                return ::pow(x, y);
            } else {
                return (TX)::powf((float)x, (float)y);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT auto log(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::logf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::log(x);
            } else {
                return ::hlog(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T floor(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::floorf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::floor(x);
            } else {
                return ::hfloor(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(const devices::math::CUDA &, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::isnanf(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::isnan(x);
            } else {
                return ::__hisnan(x);
            }
        }
    }

    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T clamp(const devices::math::CUDA&, const T x, const T min, const T max) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::fmin(max, ::fmax(x, min));
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::min(max, ::fmax(x, min));
            } else {
                return ::__hmin(max, ::__hmax(x, min));
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T min(const devices::math::CUDA&, const T a, const T b) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::fmin(a, b);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::min(a, b);
            } else {
                return ::__hmin(a, b);
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T max(const devices::math::CUDA&, const T a, const T b) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::fmax(a, b);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::max(a, b);
            } else {
                return ::__hmax(a, b);
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T abs(const devices::math::CUDA&, const T x) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::fabs(x);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::abs(x);
            } else {
                return ::__habs(x);
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T nan(const devices::math::CUDA&){
        return 0.0/0.0; //std::numeric_limits<T>::quiet_NaN();
    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T fast_tanh(const devices::math::CUDA& dev, T x) {
//        x = clamp(dev, x, -(T)3.0, (T)3.0);
//        T x_squared = x * x;
//        return x * (27 + x_squared) / (27 + 9 * x_squared);
//    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T fast_sigmoid(const devices::math::CUDA& dev, T x) {
        return fast_sigmoid(devices::math::Generic{}, x);
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T fast_tanh(const devices::math::CUDA& dev, T x) {
        return fast_tanh(devices::math::Generic{}, x);
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T atan2(const devices::math::CUDA&, const T a, const T b) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        if constexpr (utils::typing::is_same_v<T, float>) {
            return ::atan2f(a, b);
        } else {
            if constexpr (utils::typing::is_same_v<T, double>) {
                return ::atan2(a, b);
            } else {
                return (T)::atan2f((float)a, (float)b);
            }
        }
    }
    template<typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T is_finite(const devices::math::CUDA&, const T a) {
        static_assert(cuda::check<T>, "CUDA math only supports float and double");
        return true;
    }


//    // CUDA fast
//
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T sqrt(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr(utils::typing::is_same_v<T, float>){
//            return __sqrtf(x);
//        }
//        else{
//            return __sqrt(x);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T tanh(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr(utils::typing::is_same_v<T, float>){
//            return ::tanhf(x);
//        }
//        else{
//            return ::tanh(x);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T exp(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<T, float>) {
//            return __expf(x);
//        }
//        else {
//            return __exp(x);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T sin(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<T, float>){
//            return __sinf(x);
//        }
//        else{
//            return __sin(x);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T cos(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<T, float>){
//            return __cosf(x);
//        }
//        else{
//            return __cos(x);
//        }
//    }
//    template<typename TX, typename TY>
//    RL_TOOLS_FUNCTION_PLACEMENT auto pow(const devices::math::CUDA_FAST&, const TX x, const TY y) {
//        static_assert(cuda::check<TX>, "CUDA math only supports float and double");
//        static_assert(cuda::check<TY>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<TX, float> && utils::typing::is_same_v<TY, float>){
//            return __powf(x, y);
//        }
//        else{
//            return __pow(x, y);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT auto log(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<T, float>){
//            return __logf(x);
//        }
//        else{
//            return __log(x);
//        }
//    }
//    template<typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT T floor(const devices::math::CUDA_FAST&, const T x) {
//        static_assert(cuda::check<T>, "CUDA math only supports float and double");
//        if constexpr (utils::typing::is_same_v<T, float>){
//            printf("floor %f %f\n", x, __floorf(x));
//            return __floorf(x);
//        }
//        else{
//            return __floor(x);
//        }
//    }
//}
//
//
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
