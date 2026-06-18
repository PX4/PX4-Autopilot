#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CPU_GENERIC_MATH_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CPU_GENERIC_MATH_H

#include "../devices/dummy.h"

#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::math {

    template<typename T>
    T sqrt(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename T>
    T tanh(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename T>
    T exp(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename T>
    T sin(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename T>
    T cos(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename T>
    T acos(const devices::math::Dummy&, const T x) {
        return x;
    }
    template<typename TX, typename TY>
    auto pow(const devices::math::Dummy&, const TX x, const TY y) {
        return 1;
    }
    template<typename T>
    auto log(const devices::math::Dummy&, const T x) {
        return 0;
    }
    template<typename T>
    auto floor(const devices::math::Dummy&, const T x) {
        return (int)x;
    }
    template<typename T>
    auto is_nan(const devices::math::Dummy&, const T x) {
        return false;
    }
    template<typename T>
    auto is_finite(const devices::math::Dummy&, const T x) {
        return true;
    }

    template<typename T>
    T clamp(const devices::math::Dummy&, T x, T min, T max){
        return x < min ? min : (x > max ? max : x);
    }
    template<typename T>
    T min(const devices::math::Dummy&, T x, T y){
        return x < y ? x : y;
    }
    template<typename T>
    T max(const devices::math::Dummy&, T x, T y){
        return x > y ? x : y;
    }
    template<typename T>
    T abs(const devices::math::Dummy&, T x){
        return x > 0 ? x : -x;
    }
    template<typename T>
    T nan(const devices::math::Dummy&){
        return 0;
    }
    template<typename T>
    T atan2(const devices::math::Dummy&, T a, T b){
        return 0;
    }
    template<typename T>
    T fast_sigmoid(const devices::math::Dummy& dev, T x) {
        return fast_sigmoid(devices::math::Generic{}, x);
    }
    template<typename T>
    T fast_tanh(const devices::math::Dummy& dev, T x) {
        return fast_tanh(devices::math::Generic{}, x);
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
