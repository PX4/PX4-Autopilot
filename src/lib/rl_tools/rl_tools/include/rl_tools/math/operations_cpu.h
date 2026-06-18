#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_MATH_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_MATH_OPERATIONS_CPU_H

#include "operations_generic.h"

#include "../devices/cpu.h"

#include <cmath>
#include <algorithm> // required for clamp(...)
#include <stdint.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::math {
    union is_nan_struct_float{
        uint32_t i;
        float f;
    };
    union is_nan_struct_double{
        uint64_t i;
        double f;
    };
#ifdef RL_TOOLS_DISABLE_FAST_MATH
    template<typename T>
    auto is_nan(const devices::math::CPU&, const T x) {
        return std::isnan(x);
    }
#else
    template <typename T>
    bool is_nan(const devices::math::CPU, T){
        return false;
    }
    bool is_nan(const devices::math::CPU, const float x){
        is_nan_struct_float u;
        u.f = x;
        return (u.i & 0x7f800000) == 0x7f800000 && (u.i & 0x007fffff) != 0;
    }
    bool is_nan(const devices::math::CPU, const double x){
        is_nan_struct_double u;
        u.f = x;
        return (u.i & 0x7ff0000000000000) == 0x7ff0000000000000 && (u.i & 0x000fffffffffffff) != 0;
    }
#endif

    template<typename T>
    T sqrt(const devices::math::CPU&, const T x) {
        return std::sqrt(x);
    }
    template<typename T>
    T cbrt(const devices::math::CPU&, const T x) {
        return std::cbrt(x);
    }
    template<typename T>
    T tanh(const devices::math::CPU&, const T x) {
        return std::tanh(x);
    }
    template<typename T>
    T exp(const devices::math::CPU&, const T x) {
        return std::exp(x);
    }
    template<typename T>
    T sin(const devices::math::CPU&, const T x) {
        return std::sin(x);
    }
    template<typename T>
    T cos(const devices::math::CPU&, const T x) {
        return std::cos(x);
    }
    template<typename T>
    T acos(const devices::math::CPU&, const T x) {
        return std::acos(x);
    }
    template<typename TX, typename TY>
    constexpr auto pow(const devices::math::CPU&, const TX x, const TY y) {
        return std::pow(x, y);
    }
    template<typename T>
    auto log(const devices::math::CPU&, const T x) {
        return std::log(x);
    }
    template<typename T>
    auto floor(const devices::math::CPU&, const T x) {
        return std::floor(x);
    }

    template<typename T>
    auto is_finite(const devices::math::CPU&, const T x) {
        return std::isfinite(x);
    }
    template<typename T>
    T clamp(const devices::math::CPU&, T x, T min, T max){
        return std::clamp(x, min, max);
    }
    template<typename T>
    T min(const devices::math::CPU&, T x, T y){
        return std::min(x, y);
    }
    template<typename T>
    T max(const devices::math::CPU&, T x, T y){
        return std::max(x, y);
    }
    template<typename T>
    T abs(const devices::math::CPU&, T x){
        if constexpr (utils::typing::is_same_v<T, float>){
            return std::fabs(x);
        }
        else{
            if constexpr (utils::typing::is_same_v<T, double>){
                return std::abs(x);
            }
            else{
                return (T)std::abs((float)x);
            }
        }
    }
    template<typename T>
    T nan(const devices::math::CPU&){
        return std::numeric_limits<T>::quiet_NaN();
    }
    template<typename T>
    T infinity(const devices::math::CPU&){
        return std::numeric_limits<T>::infinity();
    }
    template<typename T>
    T fast_sigmoid(const devices::math::CPU& dev, T x) {
        return fast_sigmoid(devices::math::Generic{}, x);
    }
    template<typename T>
    T fast_tanh(const devices::math::CPU& dev, T x) {
        return fast_tanh(devices::math::Generic{}, x);
    }
    template<typename T>
    T atan2(const devices::math::CPU& dev, T a, T b) {
        return std::atan2(a, b);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
