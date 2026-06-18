#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_ESP32_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_ESP32_H


#include "../utils/generic/typing.h"
#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::random{
    template <typename DEV_SPEC>
    devices::random::ESP32::index_t default_engine(const devices::ESP32<DEV_SPEC>& dev, devices::random::ESP32::index_t seed = 1){
        return default_engine(devices::Generic<DEV_SPEC>{}, seed);
    };
    constexpr devices::random::ESP32::index_t next_max(const devices::random::ESP32& dev){
        return devices::random::ESP32::MAX_INDEX;
    }
    template <typename TI, typename RNG>
    auto split(const devices::random::ESP32& dev, TI split_id, RNG& rng){
        return split(devices::random::Generic<devices::math::ESP32>{}, split_id, rng);
    }
    template<typename RNG>
    void next(const devices::random::ESP32& dev, RNG& rng){
        next(devices::random::Generic<devices::math::ESP32>{}, rng);
    }

    template<typename T, typename RNG>
    T uniform_int_distribution(const devices::random::ESP32& dev, T low, T high, RNG& rng){
        return uniform_int_distribution(devices::random::Generic<devices::math::ESP32>{}, low, high, rng);
    }
    template<typename T, typename RNG>
    T uniform_real_distribution(const devices::random::ESP32& dev, T low, T high, RNG& rng){
        return uniform_real_distribution(devices::random::Generic<devices::math::ESP32>{}, low, high, rng);
    }
    namespace normal_distribution{
        template<typename T, typename RNG>
        T sample(const devices::random::ESP32& dev, T mean, T std, RNG& rng){
            return sample(devices::random::Generic<devices::math::ESP32>{}, mean, std, rng);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
