#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_RANDOM_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_RANDOM_OPERATIONS_CUDA_H


#include "../utils/generic/typing.h"
#include "operations_generic.h"

#include <curand_kernel.h>
#include "../containers/matrix/matrix.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::random{
    namespace cuda::kernels{
        template <typename DEVICE, typename SPEC>
        __global__
        void init_rng_kernel(DEVICE device, devices::random::CUDA::ENGINE<SPEC> rng, typename DEVICE::index_t seed){
            auto i = threadIdx.x + blockIdx.x * blockDim.x;
            if(i < SPEC::NUM_RNGS){
                curand_init(seed, i, 0, &get(rng.states, 0, i));
            }
        }
    }

    template<typename T, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT T uniform_real_distribution(const devices::random::CUDA& dev, T low, T high, RNG& rng){
        // static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>, "Only float and double are supported");
        if constexpr(utils::typing::is_same_v<T, float>){
            return curand_uniform(&rng) * (high - low) + low;
        }
        else{
            if constexpr(utils::typing::is_same_v<T, double>){
                return curand_uniform_double(&rng) * (high - low) + low;
            }
            else{
                return (T)curand_uniform(&rng) * (high - low) + low;
            }
        }
        return 0;
    }
    template<typename T, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT T uniform_int_distribution(const devices::random::CUDA& dev, T low, T high, RNG& rng){
        auto r = uniform_real_distribution(dev, (float)low, (float)high, rng);
        return (T)r;
    }
    namespace normal_distribution{
        template<typename T, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT T sample(const devices::random::CUDA& dev, T mean, T std, RNG& rng){
            // static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>);
            if constexpr(utils::typing::is_same_v<T, float>){
                return curand_normal(&rng) * std + mean;
            }
            else{
                if constexpr(utils::typing::is_same_v<T, double>){
                    return curand_normal_double(&rng) * std + mean;
                }
                else{
                    return ((T)curand_normal(&rng)) * std + mean;
                }
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, devices::random::CUDA::ENGINE<SPEC>& rng){
        malloc(device, rng.states);
    }
    template <typename DEVICE, typename SPEC>
    void free(DEVICE& device, devices::random::CUDA::ENGINE<SPEC>& rng){
        free(device, rng.states);
    }
    template <typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, curandState& rng, typename DEVICE::index_t seed = 1){
        using TI = typename DEVICE::index_t;
        TI id = threadIdx.x + blockIdx.x * blockDim.x;
        curand_init(seed, id, 0, &rng);
    };
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::CUDA<DEV_SPEC>& device, devices::random::CUDA::ENGINE<SPEC>& rng, typename SPEC::TI seed = 1){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::NUM_RNGS, BLOCKSIZE_COLS);
        dim3 grid(N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE_COLS);
#ifdef RL_TOOLS_DEBUG
        utils::assert_exit(device, rng.states._data != nullptr, "rng not allocated");;
#endif
        devices::cuda::TAG<DEVICE, true> tag_device{};
        random::cuda::kernels::init_rng_kernel<<<grid, block, 0, device.stream>>>(tag_device, rng, seed);
        check_status(device);
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
