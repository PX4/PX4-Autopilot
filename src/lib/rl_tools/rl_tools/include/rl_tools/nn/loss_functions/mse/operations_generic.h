#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LOSS_FUNCTIONS_MSE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LOSS_FUNCTIONS_MSE_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::loss_functions::mse{
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T evaluate(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, typename SPEC_A::T loss_weight = 1) {
        static_assert(containers::check_structure<SPEC_A, SPEC_B>);
        using T = typename SPEC_A::T;
        using TI = typename SPEC_A::TI;
        T acc = 0;
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++) {
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++) {
//                TI index = row_i * SPEC_A::COLS + col_i;
                T diff = get(a, row_i, col_i) - get(b, row_i, col_i);
                acc += diff * diff;
            }
        }
        return acc * loss_weight / (SPEC_A::ROWS * SPEC_A::COLS);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T evaluate(DEVICE& device, Tensor<SPEC_A>& a, Tensor<SPEC_B>& b, typename SPEC_A::T loss_weight = 1) {
        auto a_view = matrix_view(device, a);
        auto b_view = matrix_view(device, b);
        return evaluate(device, a_view, b_view, loss_weight);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename LOSS_WEIGHT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T evaluate(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, Tensor<LOSS_WEIGHT_SPEC>& loss_weight) {
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::LENGTH == 1);
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1);
        return evaluate(device, a, b, get(device, loss_weight, 0));
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename LOSS_WEIGHT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T evaluate(DEVICE& device, Tensor<SPEC_A>& a, Tensor<SPEC_B>& b, Tensor<LOSS_WEIGHT_SPEC>& loss_weight) {
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::LENGTH == 1);
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1);
        auto a_view = matrix_view(device, a);
        auto b_view = matrix_view(device, b);
        return evaluate(device, a, b, get(device, loss_weight, 0));
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, Matrix<SPEC_DA>& d_a, typename SPEC_A::T loss_weight) {
        static_assert(containers::check_structure<SPEC_A, SPEC_B>);
        static_assert(containers::check_structure<SPEC_A, SPEC_DA>);
        using T = typename SPEC_A::T;
        using TI = typename SPEC_A::TI;
        T constant = (T)2/((T)SPEC_A::ROWS * SPEC_A::COLS) * loss_weight;
        using T_DA = typename SPEC_DA::T;
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++) {
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++) {
//                TI index = row_i * SPEC_A::COLS + col_i;
                T diff = get(a, row_i, col_i) - get(b, row_i, col_i);
                set(d_a, row_i, col_i, static_cast<T_DA>(diff * constant));
            }
        }
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Tensor<SPEC_A>& a, Tensor<SPEC_B>& b, Tensor<SPEC_DA>& d_a, typename SPEC_A::T loss_weight){
        auto a_view = matrix_view(device, a);
        auto b_view = matrix_view(device, b);
        auto d_a_view = matrix_view(device, d_a);
        gradient(device, a_view, b_view, d_a_view, loss_weight);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA, typename LOSS_WEIGHT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, Matrix<SPEC_DA>& d_a, Tensor<LOSS_WEIGHT_SPEC>& loss_weight) {
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::LENGTH == 1);
        static_assert(LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1 || LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 0);
        constexpr bool LOSS_WEIGHT_PROVIDED = LOSS_WEIGHT_SPEC::SHAPE::template GET<0> == 1;
        typename SPEC_A::T loss_weight_value;
        if constexpr(LOSS_WEIGHT_PROVIDED) {
            loss_weight_value= get(device, loss_weight, 0);
        }
        else {
            loss_weight_value= 1;
        }
        gradient(device, a, b, d_a, loss_weight_value);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA, typename LOSS_WEIGHT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Tensor<SPEC_A>& a, Tensor<SPEC_B>& b, Tensor<SPEC_DA>& d_a, Tensor<LOSS_WEIGHT_SPEC>& loss_weight){
        auto a_view = matrix_view(device, a);
        auto b_view = matrix_view(device, b);
        auto d_a_view = matrix_view(device, d_a);
        gradient(device, a_view, b_view, d_a_view, loss_weight);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, Matrix<SPEC_DA>& d_a) {
        using TI = typename SPEC_A::TI;
        Tensor<tensor::Specification<typename SPEC_A::T, TI, tensor::Shape<TI, 0>>> dummy_loss_weight;
        gradient(device, a, b, d_a, dummy_loss_weight);
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Tensor<SPEC_A>& a, Tensor<SPEC_B>& b, Tensor<SPEC_DA>& d_a) {
        using TI = typename SPEC_A::TI;
        Tensor<tensor::Specification<typename SPEC_A::T, TI, tensor::Shape<TI, 0>>> dummy_loss_weight;
        gradient(device, a, b, d_a, dummy_loss_weight);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif