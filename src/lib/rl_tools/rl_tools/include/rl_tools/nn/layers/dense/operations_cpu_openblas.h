#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_CPU_OPENBLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_CPU_OPENBLAS_H

#include "operations_cpu_blas.h"
#include "../../../devices/cpu_openblas.h"

// Matrix
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(devices::CPU_OPENBLAS<DEV_SPEC>& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        evaluate((devices::CPU_BLAS<DEV_SPEC>&) device, layer, input, output, buffer, rng, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward((devices::CPU_BLAS<DEV_SPEC>&) device, layer, input, output, buffer, rng, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(devices::CPU_OPENBLAS<DEV_SPEC>& device, const nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward_input((devices::CPU_BLAS<DEV_SPEC> &) device, layer, d_output, d_input, buffer, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward((devices::CPU_BLAS<DEV_SPEC> &) device, layer, input, d_output, buffer, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward_full((devices::CPU_BLAS<DEV_SPEC> &) device, layer, input, d_output, d_input, buffer, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(devices::CPU_OPENBLAS<DEV_SPEC>& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        evaluate((devices::CPU_BLAS<DEV_SPEC>&) device, layer, input, output, buffer, rng, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward((devices::CPU_BLAS<DEV_SPEC>&) device, layer, input, output, buffer, rng, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(devices::CPU_OPENBLAS<DEV_SPEC>& device, const nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward_input((devices::CPU_BLAS<DEV_SPEC> &) device, layer, d_output, d_input, buffer, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward((devices::CPU_BLAS<DEV_SPEC> &) device, layer, input, d_output, buffer, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(devices::CPU_OPENBLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward_full((devices::CPU_BLAS<DEV_SPEC> &) device, layer, input, d_output, d_input, buffer, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif