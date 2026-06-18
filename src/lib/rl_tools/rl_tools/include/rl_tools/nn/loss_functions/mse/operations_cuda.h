#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LOSS_FUNCTIONS_MSE_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LOSS_FUNCTIONS_MSE_OPERATIONS_CUDA_H

#include "../../../devices/cuda.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::loss_functions::mse {
    namespace internal::mse{
        template<typename DEV_SPEC, typename SPEC_A, typename SPEC_B, typename SPEC_DA, typename LOSS_WEIGHT_SPEC>
        __global__
        void d_mse_d_x_kernel(devices::CUDA<DEV_SPEC> device, Matrix<SPEC_A> a, Matrix<SPEC_B> b, Matrix<SPEC_DA> d_a, Tensor<LOSS_WEIGHT_SPEC> loss_weight) {
            static_assert(containers::check_structure<SPEC_A, SPEC_B>);
            static_assert(containers::check_structure<SPEC_A, SPEC_DA>);
            static_assert(LOSS_WEIGHT_SPEC::SHAPE::LENGTH == 1);
            static_assert(LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1 || LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 0);
            constexpr bool LOSS_WEIGHT_PROVIDED = LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1;
            using T = typename SPEC_A::T;
            using TI = typename devices::CUDA<DEV_SPEC>::index_t;
            constexpr TI BATCH_SIZE = SPEC_A::ROWS;
            constexpr TI OUTPUT_DIM = SPEC_A::COLS;

            TI output_pos_x = blockIdx.x * blockDim.x + threadIdx.x;
            TI output_pos_y = blockIdx.y * blockDim.y + threadIdx.y;
            T loss_weight_value;
            if constexpr(LOSS_WEIGHT_PROVIDED) {
                loss_weight_value = get(device, loss_weight, 0);
            }
            else {
                loss_weight_value = 1;
            }
            if(output_pos_x < OUTPUT_DIM && output_pos_y < BATCH_SIZE){
                T diff = get(a, output_pos_y, output_pos_x) - get(b, output_pos_y, output_pos_x);
                set(d_a, output_pos_y, output_pos_x, 2*diff/(SPEC_A::ROWS * SPEC_A::COLS) * loss_weight_value);
            }
        }
    }

    template<typename DEV_SPEC, typename SPEC_A, typename SPEC_B, typename SPEC_DA, typename LOSS_WEIGHT_SPEC>
    void gradient(devices::CUDA<DEV_SPEC>& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, Matrix<SPEC_DA>& d_a, Tensor<LOSS_WEIGHT_SPEC>& loss_weight) {
        static_assert(containers::check_structure<SPEC_A, SPEC_B>);
        static_assert(containers::check_structure<SPEC_A, SPEC_DA>);
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = SPEC_A::ROWS;
        constexpr TI OUTPUT_DIM = SPEC_A::COLS;
        constexpr TI BLOCKSIZE_ACTIVATION_BATCH = 32;
        constexpr TI BLOCKSIZE_ACTIVATION_OUTPUT = 32;
        constexpr TI N_BLOCKS_ACTIVATION_BATCH = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_ACTIVATION_BATCH);
        constexpr TI N_BLOCKS_ACTIVATION_OUTPUT = RL_TOOLS_DEVICES_CUDA_CEIL(OUTPUT_DIM, BLOCKSIZE_ACTIVATION_OUTPUT);
        dim3 activation_grid(N_BLOCKS_ACTIVATION_OUTPUT, N_BLOCKS_ACTIVATION_BATCH);
        dim3 activation_block(BLOCKSIZE_ACTIVATION_OUTPUT, BLOCKSIZE_ACTIVATION_BATCH);
        using DEVICE = devices::CUDA<DEV_SPEC>;
        devices::cuda::TAG<DEVICE, true> tag_device{};
        internal::mse::d_mse_d_x_kernel<<<activation_grid, activation_block, 0, device.stream>>>(tag_device, a, b, d_a, loss_weight);
        check_status(device);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"

#endif
