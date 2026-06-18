#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_CPU_H


#include "../utils/generic/typing.h"
#include "operations_generic.h"

#include <random>
#include <limits>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC>
    void malloc(devices::CPU<DEV_SPEC>& device, devices::random::CPU::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void free(devices::CPU<DEV_SPEC>& device, devices::random::CPU::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::CPU<DEV_SPEC>& device, devices::random::CPU::ENGINE<SPEC>& rng, typename devices::CPU<DEV_SPEC>::index_t seed = 1){
        using ENGINE = devices::random::CPU::ENGINE<SPEC>;
        rng.engine = typename ENGINE::TYPE(static_cast<typename ENGINE::TYPE::result_type>(seed+1));
    };
    namespace random{
        template<typename T, typename SPEC>
        T uniform_int_distribution(const devices::random::CPU& dev, T low, T high, devices::random::CPU::ENGINE<SPEC>& rng){
            return std::uniform_int_distribution<T>(low, high)(rng.engine);
        }

        template<typename T, typename SPEC>
        T uniform_real_distribution(const devices::random::CPU& dev, T low, T high, devices::random::CPU::ENGINE<SPEC>& rng){
            static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>);
            return std::uniform_real_distribution<T>(low, high)(rng.engine);
        }
    //    template<typename T, typename RNG>
    //    const std::normal_distribution<T> standard_normal_distribution(0, 1);
        namespace normal_distribution{
            template<typename T, typename SPEC>
            T sample(const devices::random::CPU& dev, T mean, T std, devices::random::CPU::ENGINE<SPEC>& rng){
                if(std == 0){
                    return mean;
                }
                else{
                    return (T)std::normal_distribution<float>(mean, std)(rng.engine);
                }
            }
        }

    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
