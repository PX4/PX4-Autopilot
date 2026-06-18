
#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_BLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_BLAS_H

#include "tensor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    void matrix_multiply(devices::CPU_BLAS<DEV_SPEC>& device, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScoped;
#endif
        using DEVICE = devices::CPU_BLAS<DEV_SPEC>;
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{}));
        static_assert(get<1>(typename SPEC_2::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{}));
        static_assert(tensor::dense_row_major_layout<SPEC_1>());
        static_assert(tensor::dense_row_major_layout<SPEC_2>());
        static_assert(tensor::dense_row_major_layout<SPEC_OUT>());
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;

        const TI M = get<0>(typename SPEC_1::SHAPE{});
        const TI N = get<1>(typename SPEC_2::SHAPE{});
        const TI K = get<1>(typename SPEC_1::SHAPE{});

        // This operation can also handle transposed/column major matrices
        CBLAS_TRANSPOSE transA = (get<0>(typename SPEC_1::STRIDE{}) < get<1>(typename SPEC_1::STRIDE{})) ? CblasTrans : CblasNoTrans;
        CBLAS_TRANSPOSE transB = (get<0>(typename SPEC_2::STRIDE{}) < get<1>(typename SPEC_2::STRIDE{})) ? CblasTrans : CblasNoTrans;

        const TI lda = (transA == CblasNoTrans) ? get<0>(typename SPEC_1::STRIDE{}) : get<1>(typename SPEC_1::STRIDE{});
        const TI ldb = (transB == CblasNoTrans) ? get<0>(typename SPEC_2::STRIDE{}) : get<1>(typename SPEC_2::STRIDE{});
        const TI ldc = get<0>(typename SPEC_OUT::STRIDE{});

        const T alpha = 1.0;
        const T beta = 0.0;

        static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>, "CPU BLAS Tensor: matrix_multiply: Only float and double are supported for now");
        if constexpr(utils::typing::is_same_v<T, float>) {
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, t1._data, lda, t2._data, ldb, beta, result._data, ldc);
        }
        else{
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, t1._data, lda, t2._data, ldb, beta, result._data, ldc);
        }
    }
    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    void matrix_multiply_accumulate(devices::CPU_BLAS<DEV_SPEC>& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScoped; //N("tensor::matrix_multiply_accumulate");
#endif
        using DEVICE = devices::CPU_BLAS<DEV_SPEC>;
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{}));
        static_assert(get<1>(typename SPEC_2::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{}));
        static_assert(get<0>(typename SPEC_1::STRIDE{}) == 1 || get<1>(typename SPEC_1::STRIDE{}) == 1);
        static_assert(get<0>(typename SPEC_2::STRIDE{}) == 1 || get<1>(typename SPEC_2::STRIDE{}) == 1);
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        const TI M = get<0>(typename SPEC_1::SHAPE{});
        const TI N = get<1>(typename SPEC_2::SHAPE{});
        const TI K = get<1>(typename SPEC_1::SHAPE{});

        // This operation can also handle transposed/column major matrices

        CBLAS_TRANSPOSE transA = tensor::generalized_row_major<typename SPEC_1::SHAPE, typename SPEC_1::STRIDE>() ? CblasNoTrans : CblasTrans;
        CBLAS_TRANSPOSE transB = tensor::generalized_row_major<typename SPEC_2::SHAPE, typename SPEC_2::STRIDE>() ? CblasNoTrans : CblasTrans;

        const TI lda = (transA == CblasNoTrans) ? get<0>(typename SPEC_1::STRIDE{}) : get<1>(typename SPEC_1::STRIDE{});
        const TI ldb = (transB == CblasNoTrans) ? get<0>(typename SPEC_2::STRIDE{}) : get<1>(typename SPEC_2::STRIDE{});
        const TI ldc = get<0>(typename SPEC_OUT::STRIDE{});

        const T alpha = 1.0;
        const T beta = 1.0;  // We're accumulating, hence beta is 1.0
//
//        std::cout << "Shape: " << std::endl;
//        print(device, typename SPEC_1::SHAPE{});
//        std::cout << std::endl;
//        print(device, typename SPEC_2::SHAPE{});
//        std::cout << std::endl;
//        std::cout << "Stride: " << std::endl;
//        print(device, typename SPEC_1::STRIDE{});
//        std::cout << std::endl;
//        print(device, typename SPEC_2::STRIDE{});
//        std::cout << std::endl;
        static_assert(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>, "CPU BLAS Tensor: matrix_multiply: Only float and double are supported for now");
        if constexpr(utils::typing::is_same_v<T, float>){
            cblas_sgemm(CblasRowMajor, transA, transB, M, N, K, alpha, t1._data, lda, t2._data, ldb, beta, result._data, ldc);
        }
        else{
            cblas_dgemm(CblasRowMajor, transA, transB, M, N, K, alpha, t1._data, lda, t2._data, ldb, beta, result._data, ldc);
        }
    }
//        for(TI row_i=0; row_i < get<0>(typename SPEC_1::SHAPE{}); ++row_i){
//            for(TI col_j=0; col_j < get<1>(typename SPEC_2::SHAPE{}); ++col_j){
//                T acc = get(device, result, row_i, col_j);
//                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
//                    acc += get(device, t1, row_i, k) * get(device, t2, k, col_j);
//                }
//                set(device, result, acc, row_i, col_j);
//            }
//        }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
