#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_DUMMY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_DUMMY_H

#include "../devices/dummy.h"
#include "../utils/generic/typing.h"
#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC>
    void malloc(devices::Dummy<DEV_SPEC>& device, devices::random::Dummy::ENGINE<SPEC>& rng){ }
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::Dummy<DEV_SPEC>& device, devices::random::Dummy::ENGINE<SPEC>& rng, typename devices::Dummy<DEV_SPEC>::index_t seed = 1){
        rng.state = 10000 + seed;
    };
    namespace random{
        template<typename T, typename RNG>
        T uniform_int_distribution(const devices::random::Dummy& dev, T low, T high, RNG& rng){
    //        static_assert(utils::typing::is_same_v<T, typename DEVICE::index_t>);
    //        static_assert(utils::typing::is_same_v<RNG, typename DEVICE::index_t>);
    //        // https://en.wikipedia.org/wiki/Xorshift
    //        rng ^= (rng << 21);
    //        rng ^= (rng >> 35);
    //        rng ^= (rng << 4);
    //        return rng % (high-low) + low;
            return low;
        }
        template<typename T, typename RNG>
        T uniform_real_distribution(const devices::random::Dummy& dev, T low, T high, RNG& rng){
    //        static_assert(utils::typing::is_same_v<RNG, typename DEVICE::index_t>);
    //        T r = (T)uniform_int_distribution<typename DEVICE::index_t, typename DEVICE::index_t>((typename DEVICE::index_t)0, (typename DEVICE::index_t)1<<31, rng) / (T)(1<<31) - 1;
    //        return r * (high-low) + low;
            return low;
        }
        namespace normal_distribution{
            template<typename T, typename RNG>
            T sample(const devices::random::Dummy& dev, T mean, T std, RNG& rng){
                static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>);
                return 0;
            }
        }
    //        static_assert(utils::typing::is_same_v<RNG, typename DEVICE::index_t>);
    //        //
    //        T x, y, r, res;
    //        x = 0;
    //        y = 0;
    //        r = 0;
    //        res = 0;
    //
    //        do {
    //            x = (T)2.0 * (T)uniform_int_distribution<typename DEVICE::index_t, typename DEVICE::index_t>(0, 1<<31, rng) / (T)(1<<31) - 1;
    //            y = (T)2.0 * (T)uniform_int_distribution<typename DEVICE::index_t, typename DEVICE::index_t>(0, 1<<31, rng) / (T)(1<<31) - 1;
    //            r = x * x + y * y;
    //        } while (r == 0.0 || r > 1.0);
    //
    //        T d = math::sqrt(-2.0 * math::log(r) / r);
    //
    //        T n1 = x * d;
    //        T n2 = y * d;
    //
    //        res = n1 * std + mean;
    //
    //        return res;
    //        return mean; }

    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
