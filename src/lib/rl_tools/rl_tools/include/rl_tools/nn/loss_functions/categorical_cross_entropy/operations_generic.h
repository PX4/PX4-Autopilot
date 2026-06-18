#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LOSS_FUNCTIONS_CATEGORICAL_CROSS_ENTROPY_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LOSS_FUNCTIONS_CATEGORICAL_CROSS_ENTROPY_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::loss_functions::categorical_cross_entropy{
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T evaluate(DEVICE& device, Matrix<SPEC_A> a, Matrix<SPEC_B> b, typename SPEC_A::T loss_weight = 1) {
        // a is logits, b is indices resembling empirical one-hot distributions
        static_assert(SPEC_A::ROWS == SPEC_B::ROWS);
        static_assert(SPEC_B::COLS == 1);
        using T = typename SPEC_A::T;
        using TI = typename SPEC_A::TI;
        T acc = 0;
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++) {
            acc += get(a, row_i, get(b, row_i, 0));
            T maximum = max(device, row(device, a, row_i));
            acc -= maximum;
            T sum = 0;
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++){
                T logit = get(a, row_i, col_i);
                sum += math::exp(device.math, logit - maximum);
            }
            acc -= math::log(device.math, sum);
        }
        return -acc * loss_weight / SPEC_A::ROWS;
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient(DEVICE& device, Matrix<SPEC_A> a, Matrix<SPEC_B> b, Matrix<SPEC_DA> d_a, typename SPEC_A::T loss_weight = 1) {
        static_assert(SPEC_A::ROWS == SPEC_B::ROWS);
        static_assert(SPEC_B::COLS == 1);
        static_assert(containers::check_structure<SPEC_A, SPEC_DA>);
        using T = typename SPEC_A::T;
        using TI = typename SPEC_A::TI;
        T effective_weight = loss_weight / SPEC_A::ROWS;
        set_all(device, d_a, 0);
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++) {
            T maximum = max(device, row(device, a, row_i));
            T sum = 0;
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++) {
                T logit = get(a, row_i, col_i);
                sum += math::exp(device.math, logit - maximum);
            }
            T sum_reciprocal = (T)1.0 / sum;
            TI target_index = get(b, row_i, 0);
            increment(d_a, row_i, target_index, -1 * effective_weight);
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++) {
                T p = math::exp(device.math, get(a, row_i, col_i) - maximum) * sum_reciprocal;
                increment(d_a, row_i, col_i, p*effective_weight);
            }
        }
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_DA>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient_tiled(DEVICE& device, Matrix<SPEC_A> a, Matrix<SPEC_B> b, Matrix<SPEC_DA> d_a, typename SPEC_A::T loss_weight = 1) {
        static_assert(SPEC_A::ROWS == SPEC_B::ROWS);
        static_assert(SPEC_B::COLS == 1);
        static_assert(containers::check_structure<SPEC_A, SPEC_DA>);
        using T = typename SPEC_A::T;
        using TI = typename SPEC_A::TI;

        T effective_weight = loss_weight / SPEC_A::ROWS;
        set_all(device, d_a, 0);
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++) {
            T maximum = max(device, row(device, a, row_i));
            T sum = 0;
            constexpr TI TILE_SIZE = 8;
            constexpr TI REMAINDER = SPEC_A::COLS % TILE_SIZE;
            constexpr TI REMAINDER_START = SPEC_A::COLS - REMAINDER;

            for(TI tile_i = 0; tile_i < SPEC_A::COLS/TILE_SIZE; tile_i++) {
                for(TI col_i = tile_i * TILE_SIZE; col_i < (tile_i+1)*TILE_SIZE; col_i++) {
                    T logit = get(a, row_i, col_i);
                    sum += math::exp(device.math, logit - maximum);
                }
            }
            for(TI col_i = REMAINDER_START; col_i < SPEC_A::COLS; col_i++) {
                T logit = get(a, row_i, col_i);
                sum += math::exp(device.math, logit - maximum);
            }

            T sum_reciprocal = (T)1.0 / sum;

            TI target_index = get(b, row_i, 0);
            increment(d_a, row_i, target_index, -1 * effective_weight);
            for(TI tile_i = 0; tile_i < SPEC_A::COLS/TILE_SIZE; tile_i++) {
                for(TI col_i = tile_i * TILE_SIZE; col_i < (tile_i+1)*TILE_SIZE; col_i++) {
                    T p = math::exp(device.math, get(a, row_i, col_i) - maximum) * sum_reciprocal;
                    increment(d_a, row_i, col_i, p * effective_weight);
                }
            }
            for(TI col_i = REMAINDER_START; col_i < SPEC_A::COLS; col_i++) {
                T p = math::exp(device.math, get(a, row_i, col_i) - maximum) * sum_reciprocal;
                increment(d_a, row_i, col_i, p * effective_weight);
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

