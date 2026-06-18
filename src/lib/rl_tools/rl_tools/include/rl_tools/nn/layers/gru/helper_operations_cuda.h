#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_HELPER_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_HELPER_OPERATIONS_CUDA_H

#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::gru::helper{

    namespace nn::layers::gru::kernels {
        template<typename DEV_SPEC, typename SPEC_BIAS, typename SPEC_OUT>
        __global__
        void set_bias_inplace_kernel( devices::CUDA<DEV_SPEC> device, const Tensor<SPEC_BIAS> bias, Tensor<SPEC_OUT> result){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI     = typename DEVICE::index_t;
            using T      = typename SPEC_BIAS::T;

            constexpr TI ROWS = SPEC_OUT::SHAPE::template GET<0>;
            constexpr TI COLS = SPEC_OUT::SHAPE::template GET<1>;
            static_assert(SPEC_BIAS::SHAPE::template GET<0> == COLS);

            TI i = blockIdx.x * blockDim.x + threadIdx.x;
            TI j = blockIdx.y * blockDim.y + threadIdx.y;

            if (i < ROWS && j < COLS) {
                T value = get(device, bias, j);
                set(device, result, value, i, j);
            }
        }
    }

    template<typename DEV_SPEC, typename SPEC_BIAS, typename SPEC_OUT>
    void set_bias_inplace(devices::CUDA<DEV_SPEC>& device, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI     = typename DEVICE::index_t;
        static_assert(length(typename SPEC_BIAS::SHAPE{}) == 1, "bias must be 1D [HIDDEN_DIM]");
        static_assert(length(typename SPEC_OUT::SHAPE{})  == 2, "result must be 2D [BATCH_SIZE, HIDDEN_DIM]");
        static_assert(get<0>(typename SPEC_BIAS::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{}), "bias length == result second dimension (HIDDEN_DIM)");

        constexpr TI BATCH_SIZE = SPEC_OUT::SHAPE:: template GET<0>;
        constexpr TI HIDDEN_DIM = SPEC_OUT::SHAPE:: template GET<1>;
        constexpr TI BLOCK = 32;

        constexpr TI BLOCKSIZE = 32;
        constexpr TI ROWS = SPEC_OUT::SHAPE:: template GET<0>;
        constexpr TI COLS = SPEC_OUT::SHAPE:: template GET<1>;
        constexpr TI N_BLOCKS_ROWS = RL_TOOLS_DEVICES_CUDA_CEIL(ROWS, BLOCKSIZE);
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(COLS, BLOCKSIZE);
        dim3 grid(N_BLOCKS_ROWS, N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE, BLOCKSIZE);

        devices::cuda::TAG<DEVICE, true> tag_device{};
        nn::layers::gru::kernels::set_bias_inplace_kernel<<<grid, block, 0, device.stream>>>(tag_device, bias, result);
        check_status(device);
    }
    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_transpose_bias(devices::CUDA<DEV_SPEC>& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        using DEVICE = devices::CUDA<DEV_SPEC>;
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::matrix_multiply_transpose_bias");
#endif
        // Y = WX
        // Y^T = X^T W^T
        // W = t1, X^T = t2, Y^T = result
        // Y^T = result = t2 t1^T
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{})); // INPUT_DIM
        static_assert(get<0>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{})); // BATCH_SIZE
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{})); // HIDDEN_DIM
        static_assert(length(typename SPEC_BIAS::SHAPE{}) == 1);
        static_assert(get<0>(typename SPEC_BIAS::SHAPE{}) == get<0>(typename SPEC_1::SHAPE{}));
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        // for(TI i=0; i < get<0>(typename SPEC_OUT::SHAPE{}); i++){
        //     for(TI j=0; j < get<1>(typename SPEC_OUT::SHAPE{}); j++){
        //         T bias_value = get(device, bias, j);
        //         set(device, result, bias_value, i, j);
        //     }
        // }
        set_bias_inplace(device, bias, result);
        auto t1_transpose = permute(device, t1, tensor::PermutationSpec<1, 0>{});
        matrix_multiply_accumulate(device, t2, t1_transpose, result);
//        for(TI i=0; i < get<0>(typename SPEC_1::SHAPE{}); ++i){
//            for(TI j=0; j < get<0>(typename SPEC_2::SHAPE{}); ++j){
//                T acc = get(device, bias, i);
//                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
//                    acc += get(device, t1, i, k) * get(device, t2, j, k);
//                }
//                set(device, result, acc, j, i);
//            }
//        }
    }

    namespace nn::layers::gru::kernels {
        template<typename DEV_SPEC, typename SPEC_BIAS, typename SPEC_OUT>
        __global__
        void add_bias_inplace_kernel( devices::CUDA<DEV_SPEC> device, const Tensor<SPEC_BIAS> bias, Tensor<SPEC_OUT> result){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI     = typename DEVICE::index_t;
            using T      = typename SPEC_BIAS::T;

            constexpr TI HIDDEN_DIM = SPEC_BIAS::SHAPE:: template GET<0>;
            constexpr TI BATCH_SIZE = SPEC_OUT::SHAPE:: template GET<0>;

            TI i = blockIdx.x * blockDim.x + threadIdx.x;
            TI j = blockIdx.y * blockDim.y + threadIdx.y;

            if (i < BATCH_SIZE && j < HIDDEN_DIM) {
                T value = get(device, result, i, j) + get(device, bias, j);
                set(device, result, value, i, j);
            }
        }
    }

    template<typename DEV_SPEC, typename SPEC_BIAS, typename SPEC_OUT>
    void add_bias_inplace( devices::CUDA<DEV_SPEC>& device, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI     = typename DEVICE::index_t;
        static_assert(length(typename SPEC_BIAS::SHAPE{}) == 1, "bias must be 1D [HIDDEN_DIM]");
        static_assert(length(typename SPEC_OUT::SHAPE{})  == 2, "result must be 2D [BATCH_SIZE, HIDDEN_DIM]");
        static_assert(get<0>(typename SPEC_BIAS::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{}), "bias length == result second dimension (HIDDEN_DIM)");

        constexpr TI BATCH_SIZE = SPEC_OUT::SHAPE:: template GET<0>;
        constexpr TI HIDDEN_DIM = SPEC_OUT::SHAPE:: template GET<1>;
        constexpr TI BLOCK = 32;

        constexpr TI BLOCKSIZE = 32;
        constexpr TI ROWS = SPEC_OUT::SHAPE:: template GET<0>;
        constexpr TI COLS = SPEC_OUT::SHAPE:: template GET<1>;
        constexpr TI N_BLOCKS_ROWS = RL_TOOLS_DEVICES_CUDA_CEIL(ROWS, BLOCKSIZE);
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(COLS, BLOCKSIZE);
        dim3 grid(N_BLOCKS_ROWS, N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE, BLOCKSIZE);

        devices::cuda::TAG<DEVICE, true> tag_device{};
        nn::layers::gru::kernels::add_bias_inplace_kernel<<<grid, block, 0, device.stream>>>(tag_device, bias, result);
        check_status(device);
    }



    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_transpose_bias_accumulate(devices::CUDA<DEV_SPEC>& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        using DEVICE = devices::CUDA<DEV_SPEC>;
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::matrix_multiply_transpose_bias_accumulate");
#endif
        // Y^T = WX^T
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{})); // INPUT_DIM
        static_assert(get<0>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{})); // BATCH_SIZE
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{})); // HIDDEN_DIM
        static_assert(length(typename SPEC_BIAS::SHAPE{}) == 1);
        static_assert(get<0>(typename SPEC_BIAS::SHAPE{}) == get<0>(typename SPEC_1::SHAPE{}));
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        // for(TI i=0; i < get<0>(typename SPEC_OUT::SHAPE{}); i++){
        //     for(TI j=0; j < get<1>(typename SPEC_OUT::SHAPE{}); j++){
        //         T value = get(device, result, i, j) + get(device, bias, j);
        //         set(device, result, value, i, j);
        //     }
        // }
        add_bias_inplace(device, bias, result);
        auto t1_transpose = permute(device, t1, tensor::PermutationSpec<1, 0>{});
        matrix_multiply_accumulate(device, t2, t1_transpose, result);
//        for(TI i=0; i < get<0>(typename SPEC_1::SHAPE{}); ++i){
//            for(TI j=0; j < get<0>(typename SPEC_2::SHAPE{}); ++j){
//                T acc = get(device, result, j, i) + get(device, bias, i);
//                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
//                    acc += get(device, t1, i, k) * get(device, t2, j, k);
//                }
//                set(device, result, acc, j, i);
//            }
//        }
    }


    namespace nn::layers::gru::kernels{
        template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
        __global__
        void matrix_multiply_broadcast_transpose_bias(devices::CUDA<DEV_SPEC> device, const Tensor<SPEC_1> t1, const Tensor<SPEC_2> t2, const Tensor<SPEC_BIAS> bias, Tensor<SPEC_OUT> result){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename DEVICE::index_t;
            using T = typename SPEC_1::T;
            static_assert(SPEC_1::SHAPE::LENGTH == 2);
            constexpr TI ROWS = SPEC_OUT::SHAPE:: template GET<1>;
            constexpr TI COLS = SPEC_OUT::SHAPE:: template GET<0>;
            static_assert(SPEC_2::SHAPE::LENGTH == 1); // only one row
            constexpr TI INNER = SPEC_1::SHAPE:: template GET<1>;
            static_assert(INNER == SPEC_2::SHAPE:: template GET<0>);
            TI i = threadIdx.x + blockIdx.x * blockDim.x;
            TI j = threadIdx.y + blockIdx.y * blockDim.y;
            if(i < ROWS && j < COLS){
                T acc = get(device, bias, i);
                for(TI k=0; k < INNER; ++k){
                    acc += get(device, t1, i, k) * get(device, t2, k);
                }
                set(device, result, acc, j, i);
            }
        }
    }

    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_broadcast_transpose_bias(devices::CUDA<DEV_SPEC>& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        using DEVICE = devices::CUDA<DEV_SPEC>;
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::matrix_multiply_broadcast_transpose_bias");
#endif
        // Y^T = WX^T
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 1);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{})); // INPUT_DIM
//        static_assert(get<0>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{})); // BATCH_SIZE
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{})); // HIDDEN_DIM
        static_assert(length(typename SPEC_BIAS::SHAPE{}) == 1);
        static_assert(get<0>(typename SPEC_BIAS::SHAPE{}) == get<0>(typename SPEC_1::SHAPE{}));

        // using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;

        constexpr TI BLOCKSIZE = 32;
        constexpr TI ROWS = SPEC_OUT::SHAPE:: template GET<1>;
        constexpr TI COLS = SPEC_OUT::SHAPE:: template GET<0>;
        constexpr TI N_BLOCKS_ROWS = RL_TOOLS_DEVICES_CUDA_CEIL(ROWS, BLOCKSIZE);
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(COLS, BLOCKSIZE);
        dim3 grid(N_BLOCKS_ROWS, N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE, BLOCKSIZE);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        nn::layers::gru::kernels::matrix_multiply_broadcast_transpose_bias<<<grid, block, 0, device.stream>>>(tag_device, t1, t2, bias, result);
        check_status(device);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
