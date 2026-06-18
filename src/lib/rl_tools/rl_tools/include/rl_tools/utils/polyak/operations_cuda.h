#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_UTILS_POLYAK_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_UTILS_POLYAK_OPERATIONS_CUDA_H


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils::polyak {
    // todo: polyak factor as template parameter (reciprocal INT e.g.)
    namespace internal {
        template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, bool SQUARE=false>
        __global__
        void update_kernel(const Matrix<SOURCE_SPEC> source, Matrix<TARGET_SPEC> target, const typename SOURCE_SPEC::T polyak) {
            static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
            using SPEC = SOURCE_SPEC;
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            constexpr TI ROWS = SPEC::ROWS;
            constexpr TI COLS = SPEC::COLS;
            TI col_i = threadIdx.x + blockIdx.x * blockDim.x;
            TI row_i = threadIdx.y + blockIdx.y * blockDim.y;
            if(col_i < COLS && row_i < ROWS){
                T s = get(source, row_i, col_i);
                if constexpr(SQUARE){
                    s *= s;
                }
                set(target, row_i, col_i, polyak * get(target, row_i, col_i) + (1 - polyak) * s);
            }
        }
    }
    template<typename DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC, bool SQUARE=false>
    void update(devices::CUDA<DEV_SPEC>& device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target, const typename SOURCE_SPEC::T polyak) {
        static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using SPEC = SOURCE_SPEC;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_ROWS = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_COLS = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_ROWS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::ROWS, BLOCKSIZE_ROWS);
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::COLS, BLOCKSIZE_COLS);
        dim3 activation_grid(N_BLOCKS_COLS, N_BLOCKS_ROWS);
        dim3 activation_block(BLOCKSIZE_COLS, BLOCKSIZE_ROWS);
        internal::update_kernel<DEVICE, SOURCE_SPEC, TARGET_SPEC, SQUARE><<<activation_grid, activation_block, 0, device.stream>>>(source, target, polyak);
        check_status(device);
    }
    template<typename DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void update_squared(devices::CUDA<DEV_SPEC>& dev, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target, const typename SOURCE_SPEC::T polyak) {
        update<DEV_SPEC, SOURCE_SPEC, TARGET_SPEC, true>(dev, source, target, polyak);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif