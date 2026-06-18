#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_WASM32_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_WASM32_H


#include "../utils/generic/typing.h"
#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC>
    void malloc(devices::WASM32<DEV_SPEC>& device, devices::random::WASM32::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void free(devices::WASM32<DEV_SPEC>& device, devices::random::WASM32::ENGINE<SPEC>& rng){}
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::WASM32<DEV_SPEC>& device, devices::random::WASM32::ENGINE<SPEC>& rng, typename devices::WASM32<DEV_SPEC>::index_t seed = 1){
        rng.state = 10000 + seed;
    };
    namespace random{
        // template <typename DEV_SPEC>
        // devices::random::WASM32::index_t default_engine(const devices::WASM32<DEV_SPEC>& dev, devices::random::WASM32::index_t seed = 1){
        //     return default_engine(devices::random::Generic<devices::math::WASM32>{}, seed);
        // };
        // constexpr devices::random::WASM32::index_t next_max(const devices::random::WASM32& dev){
        //     return devices::random::WASM32::MAX_INDEX;
        // }
        // template <typename TI, typename RNG>
        // auto split(const devices::random::WASM32& dev, TI split_id, RNG& rng){
        //     // this operation should not alter the state of rng
        //     return split(devices::random::Generic<devices::math::WASM32>{}, split_id, rng);
        // }
        // template<typename RNG>
        // void next(const devices::random::WASM32& dev, RNG& rng){
        //     next(devices::random::Generic<devices::math::WASM32>{}, rng);
        // }

        template<typename T, typename RNG>
        T uniform_int_distribution(const devices::random::WASM32& dev, T low, T high, RNG& rng){
            return uniform_int_distribution(devices::random::Generic<devices::math::WASM32>{}, low, high, rng);
        }
        template<typename T, typename RNG>
        T uniform_real_distribution(const devices::random::WASM32& dev, T low, T high, RNG& rng){
            return uniform_real_distribution(devices::random::Generic<devices::math::WASM32>{}, low, high, rng);
        }
        namespace normal_distribution{
            template<typename T, typename RNG>
            T sample(const devices::random::WASM32& dev, T mean, T std, RNG& rng){
                return sample(devices::random::Generic<devices::math::WASM32>{}, mean, std, rng);
            }
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
