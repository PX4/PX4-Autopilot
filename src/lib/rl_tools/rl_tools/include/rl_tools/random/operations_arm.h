#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_ARM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_ARM_H


#include "../utils/generic/typing.h"
#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC>
    void malloc(devices::ARM<DEV_SPEC>& device, devices::random::ARM::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void free(devices::ARM<DEV_SPEC>& device, devices::random::ARM::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::ARM<DEV_SPEC>& device, devices::random::ARM::ENGINE<SPEC>& rng, typename devices::ARM<DEV_SPEC>::index_t seed = 1){
        rng.state = 10000 + seed;
    }
    namespace random{
        template<typename T, typename RNG>
        T uniform_int_distribution(const devices::random::ARM& dev, T low, T high, RNG& rng){
            return uniform_int_distribution(devices::random::Generic<devices::math::ARM>{}, low, high, rng);
        }
        template<typename T, typename RNG>
        T uniform_real_distribution(const devices::random::ARM& dev, T low, T high, RNG& rng){
            return uniform_real_distribution(devices::random::Generic<devices::math::ARM>{}, low, high, rng);
        }
        namespace normal_distribution{
            template<typename T, typename RNG>
            T sample(const devices::random::ARM& dev, T mean, T std, RNG& rng){
                return sample(devices::random::Generic<devices::math::ARM>{}, mean, std, rng);
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
