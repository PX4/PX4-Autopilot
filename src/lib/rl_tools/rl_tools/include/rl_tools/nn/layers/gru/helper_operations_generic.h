#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_HELPER_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_HELPER_OPERATIONS_GENERIC_H

#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::gru::helper{
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_transpose_bias(DEVICE& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
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
        for(TI i=0; i < get<0>(typename SPEC_OUT::SHAPE{}); i++){
            for(TI j=0; j < get<1>(typename SPEC_OUT::SHAPE{}); j++){
                T bias_value = get(device, bias, j);
                set(device, result, bias_value, i, j);
            }
        }
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
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_transpose_bias_accumulate(DEVICE& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
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
        for(TI i=0; i < get<0>(typename SPEC_OUT::SHAPE{}); i++){
            for(TI j=0; j < get<1>(typename SPEC_OUT::SHAPE{}); j++){
                T value = get(device, result, i, j) + get(device, bias, j);
                set(device, result, value, i, j);
            }
        }
        auto t1_transpose = permute(device, t1, tensor::PermutationSpec<1, 0>{});
        matrix_multiply_accumulate(device, t2, t1_transpose, result);
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_BIAS, typename SPEC_OUT>
    void matrix_multiply_broadcast_transpose_bias(DEVICE& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, const Tensor<SPEC_BIAS>& bias, Tensor<SPEC_OUT>& result){
        // t2_expand = expand_along_cols(t2)
        // result^T = t1 t2_expand + bias
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
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        for(TI i=0; i < get<1>(typename SPEC_OUT::SHAPE{}); ++i){
            for(TI j=0; j < get<0>(typename SPEC_OUT::SHAPE{}); ++j){
                T acc = get(device, bias, i);
                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
                    acc += get(device, t1, i, k) * get(device, t2, k);
                }
                set(device, result, acc, j, i);
            }
        }
    }

    namespace unary_operations{
        template <typename DEVICE, typename PARAMETER, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T d_sigmoid(DEVICE& device, const PARAMETER& parameter, T a){
            T s = sigmoid(device, parameter, a);
            return s * (1 - s);
        }
        template <typename DEVICE, typename PARAMETER, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T d_sigmoid_post_activation(DEVICE& device, const PARAMETER& parameter, T s){
            return s * (1 - s);
        }
        template <typename DEVICE, typename PARAMETER, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T d_tanh(DEVICE& device, const PARAMETER& parameter, T a){
            T t = tanh(device, parameter, a);// todo: make version that can take advantage of a stored value so that tanh does not have to be calculated again
            return 1 - t * t;
        }
        template <typename DEVICE, typename PARAMETER, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T d_tanh_post_activation(DEVICE& device, const PARAMETER& parameter, T t){
            return 1 - t * t;
        }
    }
    template<typename DEVICE, typename SPEC, typename SPEC_OUTPUT>
    void d_sigmoid(DEVICE& device, Tensor<SPEC>& t, Tensor<SPEC_OUTPUT>& output){
        static_assert(tensor::same_dimensions<SPEC, SPEC_OUTPUT>());
        using T = typename SPEC::T;
        unary_operation(device, tensor::OperationLegacy<unary_operations::d_sigmoid<DEVICE, tensor::OperationEmptyParameter, T>, tensor::OperationEmptyParameter>{}, t, output);
    }
    template<typename DEVICE, typename SPEC, typename SPEC_OUTPUT>
    void d_sigmoid_post_activation(DEVICE& device, Tensor<SPEC>& t){
        // this function takes sigmoid(x) as an input instead of x (to reuse it in the calculation)
        static_assert(tensor::same_dimensions<SPEC, SPEC_OUTPUT>());
        using T = typename SPEC::T;
        unary_operation(device, tensor::OperationLegacy<unary_operations::d_sigmoid_post_activation<DEVICE, tensor::OperationEmptyParameter, T>, tensor::OperationEmptyParameter>{}, t);
    }
    template<typename DEVICE, typename SPEC, typename SPEC_OUTPUT>
    void d_sigmoid_post_activation(DEVICE& device, Tensor<SPEC>& t, Tensor<SPEC_OUTPUT>& output){
        // this function takes sigmoid(x) as an input instead of x (to reuse it in the calculation)
        static_assert(tensor::same_dimensions<SPEC, SPEC_OUTPUT>());
        using T = typename SPEC::T;
        unary_operation(device, tensor::OperationLegacy<unary_operations::d_sigmoid_post_activation<DEVICE, tensor::OperationEmptyParameter, T>, tensor::OperationEmptyParameter>{}, t, output);
    }
    template<typename DEVICE, typename SPEC, typename SPEC_OUTPUT>
    void d_tanh(DEVICE& device, Tensor<SPEC>& t, Tensor<SPEC_OUTPUT>& output){
        static_assert(tensor::same_dimensions<SPEC, SPEC_OUTPUT>());
        using T = typename SPEC::T;
        unary_operation(device, tensor::OperationLegacy<unary_operations::d_tanh<DEVICE, tensor::OperationEmptyParameter, T>, tensor::OperationEmptyParameter>{}, t, output);
    }
    template<typename DEVICE, typename SPEC, typename SPEC_OUTPUT>
    void d_tanh_post_activation(DEVICE& device, Tensor<SPEC>& t, Tensor<SPEC_OUTPUT>& output){
        // this function takes tanh(x) as an input instead of x (to reuse it in the calculation)
        static_assert(tensor::same_dimensions<SPEC, SPEC_OUTPUT>());
        using T = typename SPEC::T;
        unary_operation(device, tensor::OperationLegacy<unary_operations::d_tanh_post_activation<DEVICE, tensor::OperationEmptyParameter, T>, tensor::OperationEmptyParameter>{}, t, output);
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
