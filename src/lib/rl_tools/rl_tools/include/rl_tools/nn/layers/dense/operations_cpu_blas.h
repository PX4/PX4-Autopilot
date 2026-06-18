#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_CPU_BLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_CPU_BLAS_H

#include "operations_cpu.h"
#include "../../../utils/generic/memcpy.h"
#include "../../../devices/cpu_blas.h"

// extern "C" {
//     void cblas_sgemm(const enum CBLAS_ORDER Order, const enum CBLAS_TRANSPOSE TransA, const enum CBLAS_TRANSPOSE TransB, const int M, const int N, const int K, const float alpha, const float *A, const int lda, const float *B, const int ldb, const float beta, float *C, const int ldc);
//     void cblas_dgemm(const enum CBLAS_ORDER Order, const enum CBLAS_TRANSPOSE TransA, const enum CBLAS_TRANSPOSE TransB, const int M, const int N, const int K, const double alpha, const double *A, const int lda, const double *B, const int ldb, const double beta, double *C, const int ldc);
// }



RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace nn::layers::dense{
        template <typename LAYER_TYPE, typename INPUT_SPEC, typename OUTPUT_SPEC>
        struct CHECK_FORMATS{
            using PARAMETER_TYPE = typename decltype(LAYER_TYPE::weights.parameters)::T;
            using INPUT_TYPE = typename INPUT_SPEC::T;
            using OUTPUT_TYPE = typename OUTPUT_SPEC::T;
            static constexpr bool UNIFORM_TYPES = utils::typing::is_same_v<PARAMETER_TYPE, INPUT_TYPE> && utils::typing::is_same_v<PARAMETER_TYPE, OUTPUT_TYPE>;
            static constexpr bool UNIFORM_FLOAT_OR_DOUBLE = UNIFORM_TYPES && (utils::typing::is_same_v<PARAMETER_TYPE, float> || utils::typing::is_same_v<PARAMETER_TYPE, double>);
#if defined(RL_TOOLS_NUMERIC_TYPES_ENABLE_BF16) and defined(RL_TOOLS_BACKEND_ENABLE_MKL)
            static constexpr bool MIXED = utils::typing::is_same_v<PARAMETER_TYPE, numeric_types::bf16> && utils::typing::is_same_v<INPUT_TYPE, numeric_types::bf16> && utils::typing::is_same_v<OUTPUT_TYPE, float>;
            static constexpr bool VALUE = UNIFORM_FLOAT_OR_DOUBLE || MIXED;
#else
            static constexpr bool VALUE = UNIFORM_FLOAT_OR_DOUBLE;
#endif
        };
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, INPUT_SPEC, OUTPUT_SPEC>::VALUE>>
    void evaluate(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using WEIGHT_TYPE = typename decltype(layer.weights.parameters)::T;
        using BIAS_TYPE = typename decltype(layer.biases.parameters)::T;
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, BIAS_TYPE>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename INPUT_SPEC::T>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename OUTPUT_SPEC::T>);

        // Warning do not use the same buffer for input and output!
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        using DEVICE = devices::CPU_BLAS<DEV_SPEC>;
        using T = typename INPUT_SPEC::T;
        using TI = typename DEVICE::index_t;

        constexpr T alpha = 1;
        constexpr T beta = 1;
        // op(A) m x k = input     (B x I)
        // op(B) k x n = weights^T (I x O)
        // op(C) m x n = OUTPUT    (B x O)
        constexpr auto m = BATCH_SIZE;
        constexpr auto k = LAYER_SPEC::INPUT_DIM;
        constexpr auto n = LAYER_SPEC::OUTPUT_DIM;

        set_broadcast(device, matrix_view(device, layer.biases.parameters), output);

        if constexpr(utils::typing::is_same_v<T, float>){
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, input._data, row_pitch(input), layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, output._data, row_pitch(output));
        }
        else{
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, input._data, row_pitch(input), layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, output._data, row_pitch(output));
        }
        for(TI i = 0; i < BATCH_SIZE; i++){
            for(TI j = 0; j < LAYER_SPEC::OUTPUT_DIM; j++){
                set(output, i, j, activation<typename DEVICE::SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(get(output, i, j)));
            }
        }
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, INPUT_SPEC, OUTPUT_SPEC>::VALUE>>
    void forward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using WEIGHT_TYPE = typename decltype(layer.weights.parameters)::T;
        using OUTPUT_TYPE = typename OUTPUT_SPEC::T;
        using BIAS_TYPE = typename decltype(layer.biases.parameters)::T;
        // Warning do not use the same buffer for input and output!
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        using CHECK = nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, INPUT_SPEC, OUTPUT_SPEC>;
        using T = WEIGHT_TYPE;
        using TI = typename devices::CPU_BLAS<DEV_SPEC>::index_t;

        constexpr T alpha = 1;
        constexpr T beta = 1;
        // op(A) m x k = input     (B x I)
        // op(B) k x n = weights^T (I x O)
        // op(C) m x n = OUTPUT    (B x O)
        constexpr auto m = BATCH_SIZE;
        constexpr auto k = LAYER_SPEC::INPUT_DIM;
        constexpr auto n = LAYER_SPEC::OUTPUT_DIM;


        set_broadcast(device, matrix_view(device, layer.biases.parameters), output);

        if constexpr(utils::typing::is_same_v<T, float>){
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, (T*)input._data, row_pitch(input), (T*)layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, (T*)output._data, row_pitch(output));
        }
        else{
            if constexpr(utils::typing::is_same_v<T, double>){
                cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, (T*)input._data, row_pitch(input), (T*)layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, (T*)output._data, row_pitch(output));
            }
#ifdef RL_TOOLS_BACKEND_ENABLE_MKL
            else{
                cblas_gemm_bf16bf16f32(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, (MKL_BF16*)input._data, row_pitch(input), (MKL_BF16*)layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, (float*)output._data, row_pitch(output));
            }
#endif
        }
        copy(device, device, output, layer.pre_activations);
        for(TI i = 0; i < BATCH_SIZE; i++){
            for(TI j = 0; j < LAYER_SPEC::OUTPUT_DIM; j++){
                set(output, i, j, activation<typename DEV_SPEC::MATH, OUTPUT_TYPE, LAYER_SPEC::ACTIVATION_FUNCTION>(get(output, i, j)));
            }
        }
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_PRE_ACTIVATIONS_SPEC>
    void backward_pre_activations(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_PRE_ACTIVATIONS_SPEC>& d_pre_activations, nn::layers::dense::Buffer&) {
        // calculating pre-activation
        using LAYER = nn::layers::dense::LayerBackward<LAYER_SPEC>;
        constexpr auto OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        static_assert(D_OUTPUT_SPEC::COLS == OUTPUT_DIM);
        static_assert(D_PRE_ACTIVATIONS_SPEC::COLS == OUTPUT_DIM);
        static_assert(LAYER::INTERNAL_BATCH_SIZE == D_OUTPUT_SPEC::ROWS);
        static_assert(LAYER::INTERNAL_BATCH_SIZE == D_PRE_ACTIVATIONS_SPEC::ROWS);
        constexpr auto BATCH_SIZE = D_PRE_ACTIVATIONS_SPEC::ROWS;
        using T = typename D_PRE_ACTIVATIONS_SPEC::T;
        using TI = typename devices::CPU_BLAS<DEV_SPEC>::index_t;
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < OUTPUT_DIM; output_i++) {
                T d_pre_activation = d_activation_d_x<typename DEV_SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, output_i)) * get(d_output, batch_i, output_i);
                set(d_pre_activations, batch_i, output_i, d_pre_activation);
            }
        }
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_PRE_ACTIVATIONS_SPEC>
    void backward_pre_activations(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_PRE_ACTIVATIONS_SPEC>& d_pre_activations) {
        nn::layers::dense::Buffer buffer;
        backward_pre_activations(device, layer, d_output, d_pre_activations, buffer);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, INPUT_SPEC, D_OUTPUT_SPEC>::VALUE>>
    void backward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        // Warning do not reuse d_output as d_output is used as a temporary buffer
        // todo: create sparate function that does not set d_input (to save cost on backward pass for the first layer)
        // todo: think about storing gradient in column major order to avoid iterating over the minor dimension
        using WEIGHT_TYPE = typename decltype(layer.weights.parameters)::T;
        using BIAS_TYPE = typename decltype(layer.biases.parameters)::T;
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, BIAS_TYPE>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename INPUT_SPEC::T>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename D_OUTPUT_SPEC::T>);
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        using T = WEIGHT_TYPE;
        using TI = typename devices::CPU_BLAS<DEV_SPEC>::index_t;

        {
            // d_weights
            constexpr T alpha = 1;
            constexpr T beta = 1;
            // op(A) m x k = d_output^T (O x B)
            // op(B) k x n = input      (B x I)
            // op(C) m x n = d_weights  (O x I)

            constexpr auto m = LAYER_SPEC::OUTPUT_DIM;
            constexpr auto k = BATCH_SIZE;
            constexpr auto n = LAYER_SPEC::INPUT_DIM;

            // the following is not calling backprop_pre_activations because we can fuse it with accumulating the bias gradient
            for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
                for(TI output_i = 0; output_i < OUTPUT_DIM; output_i++) {
                    T d_pre_activation = d_activation_d_x<typename DEV_SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, output_i)) * get(d_output, batch_i, output_i);
                    increment(device, layer.biases.gradient, d_pre_activation, output_i);
                    set(d_output, batch_i, output_i, d_pre_activation);
                }
            }

            if constexpr(utils::typing::is_same_v<T, float>){
                cblas_sgemm(CblasRowMajor, CblasTrans, CblasNoTrans, m, n, k, alpha, (T*)d_output._data, row_pitch(d_output), (T*)input._data, row_pitch(input), beta, (T*)layer.weights.gradient._data, decltype(layer.weights.gradient)::SPEC::STRIDE::FIRST);
            }
            else{
                cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, m, n, k, alpha, (T*)d_output._data, row_pitch(d_output), (T*)input._data, row_pitch(input), beta, (T*)layer.weights.gradient._data, decltype(layer.weights.gradient)::SPEC::STRIDE::FIRST);
            }
        }
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_PRE_ACTIVATIONS_SPEC, typename D_INPUT_SPEC, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, D_INPUT_SPEC, D_PRE_ACTIVATIONS_SPEC>::VALUE>>
    void backward_input_additional(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<D_PRE_ACTIVATIONS_SPEC>& d_pre_activaitons, Matrix<D_INPUT_SPEC>& d_input) {
        // ATTENTION: this requires d_pre_activation as inputs and not d_output!!
        using WEIGHT_TYPE = typename decltype(layer.weights.parameters)::T;
        using BIAS_TYPE = typename decltype(layer.biases.parameters)::T;
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, BIAS_TYPE>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename D_INPUT_SPEC::T>);
        static_assert(utils::typing::is_same_v<WEIGHT_TYPE, typename D_PRE_ACTIVATIONS_SPEC::T>);
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_PRE_ACTIVATIONS_SPEC>);
        constexpr auto INPUT_DIM = LAYER_SPEC::INPUT_DIM;
        constexpr auto OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr auto BATCH_SIZE = D_PRE_ACTIVATIONS_SPEC::ROWS;
        using T = WEIGHT_TYPE;
        using TI = typename devices::CPU_BLAS<DEV_SPEC>::index_t;
        // d_input
        constexpr T alpha = 1;
        constexpr T beta = 0;
        // op(A) m x k = d_output   (B x O)
        // op(B) k x n = weights    (O x I)
        // op(C) m x n = d_input    (B x I)

        constexpr auto m = BATCH_SIZE;
        constexpr auto k = LAYER_SPEC::OUTPUT_DIM;
        constexpr auto n = LAYER_SPEC::INPUT_DIM;

        if constexpr(utils::typing::is_same_v<T, float>){
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m, n, k, alpha, (T*)d_pre_activaitons._data, row_pitch(d_pre_activaitons), (T*)layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, (T*)d_input._data, row_pitch(d_input));
        }
        else{
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m, n, k, alpha, (T*)d_pre_activaitons._data, row_pitch(d_pre_activaitons), (T*)layer.weights.parameters._data, decltype(layer.weights.parameters)::SPEC::STRIDE::FIRST, beta, (T*)d_input._data, row_pitch(d_input));
        }
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, D_INPUT_SPEC, D_OUTPUT_SPEC>::VALUE>>
    void backward_input(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward_pre_activations(device, layer, d_output, d_output);
        backward_input_additional(device, layer, d_output, d_input);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>, typename = typename utils::typing::enable_if_t<nn::layers::dense::CHECK_FORMATS<nn::layers::dense::LayerForward<LAYER_SPEC>, D_INPUT_SPEC, D_OUTPUT_SPEC>::VALUE>>
    void backward_full(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward(device, layer, input, d_output, buffer, mode);
        backward_input_additional(device, layer, d_output, d_input);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void evaluate(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, layer, matrix_view_input, layer.output, buffer, rng);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    void forward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEV_SPEC, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    void backward_input(devices::CPU_BLAS<DEV_SPEC>& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, layer, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    void backward(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, layer, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEV_SPEC, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    void backward_full(devices::CPU_BLAS<DEV_SPEC>& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, layer, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif