#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_STANDARDIZE_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_STANDARDIZE_OPERATIONS_CUDA_H
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"

#include "../../../nn/nn.h"
#include "../../../nn/parameters/parameters.h"
#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace nn::layers::standardize::cuda::kernel{
        template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
        __global__ void evaluate(devices::CUDA<DEV_SPEC> device, const nn::layers::standardize::LayerForward<LAYER_SPEC> layer, const Matrix<INPUT_SPEC> input, Matrix<OUTPUT_SPEC> output) {
            static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using T = typename LAYER_SPEC::T;
            using TI = typename DEVICE::index_t;
            constexpr TI DIM = LAYER_SPEC::INPUT_DIM;
            constexpr TI BATCH_SIZE = INPUT_SPEC::ROWS;
            TI output_i = blockIdx.x * blockDim.x + threadIdx.x;
            TI batch_i = blockIdx.y * blockDim.y + threadIdx.y;
            if(output_i < DIM && batch_i < BATCH_SIZE){
                T mean = get(layer.mean.parameters, 0, output_i);
                T precision = get(layer.precision.parameters, 0, output_i);
                T input_value = get(input, batch_i, output_i);
                T output_value = (input_value - mean) * precision;
                set(output, batch_i, output_i, output_value);
            }
        }
        template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC>
        __global__ void backward_input(devices::CUDA<DEV_SPEC> device, nn::layers::standardize::LayerBackward<LAYER_SPEC> layer, const Matrix<D_OUTPUT_SPEC> d_output, Matrix<D_INPUT_SPEC> d_input){
            static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using T = typename LAYER_SPEC::T;
            using TI = typename DEVICE::index_t;
            constexpr TI DIM = LAYER_SPEC::INPUT_DIM;
            constexpr TI BATCH_SIZE = D_INPUT_SPEC::ROWS;
            TI output_i = blockIdx.x * blockDim.x + threadIdx.x;
            TI batch_i = blockIdx.y * blockDim.y + threadIdx.y;
            if(output_i < DIM && batch_i < BATCH_SIZE){
                T d_output_value = get(d_output, batch_i, output_i);
                T precision = get(layer.precision.parameters, 0, output_i);
                T d_input_value = d_output_value * precision;
                set(d_input, batch_i, output_i, d_input_value);
            }
        }
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void evaluate(devices::CUDA<DEV_SPEC>& device, const nn::layers::standardize::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename LAYER_SPEC::T;
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
        constexpr TI BATCH_SIZE = INPUT_SPEC::ROWS;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_ACTIVATION_BATCH = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_ACTIVATION_OUTPUT = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_ACTIVATION_BATCH = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_ACTIVATION_BATCH);
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_ACTIVATION_OUTPUT = RL_TOOLS_DEVICES_CUDA_CEIL(LAYER_SPEC::OUTPUT_DIM, BLOCKSIZE_ACTIVATION_OUTPUT);
        dim3 activation_grid(N_BLOCKS_ACTIVATION_OUTPUT, N_BLOCKS_ACTIVATION_BATCH);
        dim3 activation_block(BLOCKSIZE_ACTIVATION_OUTPUT, BLOCKSIZE_ACTIVATION_BATCH);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        nn::layers::standardize::cuda::kernel::evaluate<<<activation_grid, activation_block, 0, device.stream>>>(tag_device, layer, input, output);
        check_status(device);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        nn::layers::standardize::Buffer buffer;
        evaluate(device, layer, input, output, buffer, rng);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, typename decltype(layer.output)::SPEC>);
        forward(device, layer, input, layer.output, rng);
    }
//    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG>
//    void forward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, RNG& rng) {
//        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
//        forward(device, layer, input, layer.output, rng);
//        copy(device, device, layer.output, output);
//    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    void backward_input(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(LAYER_SPEC::INPUT_DIM == LAYER_SPEC::OUTPUT_DIM);
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI INPUT_DIM = LAYER_SPEC::INPUT_DIM;
        constexpr TI OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr TI BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        using T = typename LAYER_SPEC::T;

        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_ACTIVATION_BATCH = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t BLOCKSIZE_ACTIVATION_OUTPUT = 32;
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_ACTIVATION_BATCH = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_ACTIVATION_BATCH);
        constexpr typename devices::CUDA<DEV_SPEC>::index_t N_BLOCKS_ACTIVATION_OUTPUT = RL_TOOLS_DEVICES_CUDA_CEIL(LAYER_SPEC::OUTPUT_DIM, BLOCKSIZE_ACTIVATION_OUTPUT);
        dim3 activation_grid(N_BLOCKS_ACTIVATION_OUTPUT, N_BLOCKS_ACTIVATION_BATCH);
        dim3 activation_block(BLOCKSIZE_ACTIVATION_OUTPUT, BLOCKSIZE_ACTIVATION_BATCH);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        nn::layers::standardize::cuda::kernel::backward_input<<<activation_grid, activation_block, 0, device.stream>>>(tag_device, layer, d_output, d_input);
        check_status(device);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    void backward(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    void backward_full(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // this is the same as the standardize layer does not have trainable parameters
        backward_input(device, layer, d_output, d_input, mode);
    }
    template<typename DEV_SPEC, typename SPEC>
    void zero_gradient(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<SPEC>& layer) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }
    template<typename DEV_SPEC, typename SPEC, typename OPTIMIZER>
    void update(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        // this is a no-op as the standardize layer does not have trainable parameters
    }

    template<typename DEV_SPEC, typename SPEC, typename OPTIMIZER>
    void _reset_optimizer_state(devices::CUDA<DEV_SPEC>& device, nn::layers::standardize::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }
//    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
//    void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::standardize::LayerForward<SOURCE_SPEC>& source, nn::layers::standardize::LayerForward<TARGET_SPEC>& target){
//        nn::layers::standardize::check_compatibility<SOURCE_SPEC, TARGET_SPEC>;
//        copy(source_device, target_device, source.mean, target.mean);
//        copy(source_device, target_device, source.precision, target.precision);
//    }
//    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
//    void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::standardize::LayerGradient<SOURCE_SPEC>& source, nn::layers::standardize::LayerGradient<TARGET_SPEC>& target){
//        nn::layers::standardize::check_compatibility<SOURCE_SPEC, TARGET_SPEC>;
//        copy(source_device, target_device, static_cast<const nn::layers::standardize::LayerForward<SOURCE_SPEC>&>(source), static_cast<nn::layers::standardize::LayerForward<TARGET_SPEC>&>(target));
//        copy(source_device, target_device, source.output, target.output);
//    }
    template <typename DEV_SPEC, typename SPEC_1, typename SPEC_2>
    typename SPEC_1::T abs_diff(devices::CUDA<DEV_SPEC>& device, const rl_tools::nn::layers::standardize::LayerForward<SPEC_1>& l1, const rl_tools::nn::layers::standardize::LayerForward<SPEC_2>& l2) {
        nn::layers::standardize::check_compatibility<SPEC_1, SPEC_2>;
        using T = typename SPEC_1::T;
        T acc = 0;
        acc += abs_diff(device, l1.mean, l2.mean);
        acc += abs_diff(device, l1.precision, l2.precision);
        return acc;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
