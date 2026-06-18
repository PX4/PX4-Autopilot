#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_ESP32_OPT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_ESP32_OPT_H

#include "../../../../devices/esp32.h"
#include "../../../../nn/layers/dense/layer.h"
#include "../../../../nn/mode.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void evaluate(devices::esp32::OPT<DEV_SPEC>& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // For performance reasons: restricted to dense row-major matrices (row-pitch is allowed)
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        static_assert(INPUT_SPEC::COL_PITCH == 1);
        static_assert(OUTPUT_SPEC::COL_PITCH == 1);
        static_assert(decltype(layer.weights.parameters)::COL_PITCH == 1);
        static_assert(decltype(layer.biases.parameters)::COL_PITCH == 1);
//        static_assert(utils::typing::is_same_v<typename LAYER_SPEC::T, float>);

        // Warning do not use the same buffer for input and output!
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        // static_assert(BATCH_SIZE == 1);
        using DEVICE = devices::esp32::OPT<DEV_SPEC>;
        using T = typename LAYER_SPEC::T;
        {

            T *weights_row;
            T *input_row = input._data;
            T *output_row = output._data;

            T *weights_element, *biases_element, *input_element, *output_element;

            T acc;
            uint32_t weights_row_i, batch_i = BATCH_SIZE, input_i;

            {
                do{
                    output_element = output_row;
                    biases_element = layer.biases.parameters._data;

                    weights_row_i = LAYER_SPEC::OUTPUT_DIM;
                    weights_row = layer.weights.parameters._data;

                    do{
                        acc = 0.0f;
                        input_element = input_row;
                        weights_element = weights_row;

                        // reduction
                        input_i = ((uint32_t)LAYER_SPEC::INPUT_DIM) >> 2U;
                        while (input_i > 0U){
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;

                            input_i--;
                        }
                        input_i = ((uint32_t)LAYER_SPEC::INPUT_DIM) % 0x4U;
                        while (input_i > 0U){
                            acc += *weights_element++ * *input_element++;
                            input_i--;
                        }

                        acc += *biases_element++;
                        acc = activation<typename DEVICE::SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(acc);

                        *output_element++ = acc;

                        weights_row_i--;

                        weights_row += decltype(layer.weights.parameters)::ROW_PITCH;

                    }while (weights_row_i > 0U);

                    output_row += OUTPUT_SPEC::ROW_PITCH;
                    input_row += INPUT_SPEC::ROW_PITCH;
                    batch_i--;
                }while (batch_i > 0U);
            }
        }
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::esp32::OPT<DEV_SPEC>& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // For performance reasons: restricted to dense row-major matrices (row-pitch is allowed)
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        static_assert(INPUT_SPEC::COL_PITCH == 1);
        static_assert(OUTPUT_SPEC::COL_PITCH == 1);
        static_assert(decltype(layer.weights.parameters)::COL_PITCH == 1);
        static_assert(decltype(layer.biases.parameters)::COL_PITCH == 1);
//        static_assert(utils::typing::is_same_v<typename LAYER_SPEC::T, float>);

        // Warning do not use the same buffer for input and output!
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        // static_assert(BATCH_SIZE == 1);
        using DEVICE = devices::esp32::OPT<DEV_SPEC>;
        using T = typename LAYER_SPEC::T;
        using TI = typename DEVICE::index_t;
        {

            T *weights_row;
            T *input_row = input._data;
            T *pre_activations_row = layer.pre_activations._data;
            T *output_row = output._data;

            T *weights_element, *biases_element, *input_element, *pre_activations_element, *output_element;

            T acc;
            TI weights_row_i, batch_i = BATCH_SIZE, input_i;

            {
                do{
                    pre_activations_element = pre_activations_row;
                    output_element = output_row;
                    biases_element = layer.biases.parameters._data;

                    weights_row_i = LAYER_SPEC::OUTPUT_DIM;
                    weights_row = layer.weights.parameters._data;

                    do{
                        acc = 0.0f;
                        input_element = input_row;
                        weights_element = weights_row;

                        // reduction
                        input_i = ((TI)LAYER_SPEC::INPUT_DIM) >> 2U;
                        while (input_i > 0U){
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;
                            acc += *weights_element++ * *input_element++;

                            input_i--;
                        }
                        input_i = ((TI)LAYER_SPEC::INPUT_DIM) % 0x4U;
                        while (input_i > 0U){
                            acc += *weights_element++ * *input_element++;
                            input_i--;
                        }

                        acc += *biases_element++;
                        *pre_activations_element++ = acc;
                        acc = activation<typename DEVICE::SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(acc);

                        *output_element++ = acc;

                        weights_row_i--;

                        weights_row += decltype(layer.weights.parameters)::ROW_PITCH;

                    }while (weights_row_i > 0U);

                    pre_activations_row += OUTPUT_SPEC::ROW_PITCH;
                    output_row += OUTPUT_SPEC::ROW_PITCH;
                    input_row += INPUT_SPEC::ROW_PITCH;
                    batch_i--;
                }while (batch_i > 0U);
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif