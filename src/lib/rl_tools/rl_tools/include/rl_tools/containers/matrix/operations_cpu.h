#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_H

#include "operations_generic.h"

#include <iostream>
#include <iomanip>
// #include <immintrin.h> // For AVX intrinsics
#include <cstdlib>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    void print(DEVICE& device, const Matrix<SPEC>& m){
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                std::cout << std::fixed << std::setw(12) << std::setprecision(6) << get(m, row_i, col_i) << " ";
            }
            std::cout << std::endl;
        }
    }
    template<typename DEVICE, typename SPEC>
    void print_python_literal(DEVICE& device, const Matrix<SPEC>& m){
        std::cout << "[" << std::endl;
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            std::cout << "    [";
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                std::cout << std::fixed << std::setw(12) << std::setprecision(6) << get(m, row_i, col_i);
                if(col_i < SPEC::COLS - 1){
                    std::cout << ", ";
                }
            }
            std::cout << "],";
            std::cout << std::endl;
        }
        std::cout << "]" << std::endl;
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_view(devices::CPU<SOURCE_DEV_SPEC>& source_device, devices::CPU<TARGET_DEV_SPEC>& target_device, const Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        using SOURCE_DEVICE = devices::CPU<SOURCE_DEV_SPEC>;
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using SPEC = SPEC_1;
        vectorize_unary<SOURCE_DEVICE, SPEC_1, SPEC_2, containers::vectorization::operators::copy<typename SOURCE_DEVICE::SPEC::MATH, typename SPEC::T>>(source_device, source, target);
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(devices::CPU<SOURCE_DEV_SPEC>& source_device, devices::CPU<TARGET_DEV_SPEC>& target_device, const Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        using SOURCE_DEVICE = devices::CPU<SOURCE_DEV_SPEC>;
        using TI = typename SOURCE_DEVICE::index_t;
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        if constexpr(containers::check_memory_layout<SPEC_1, SPEC_2>){
            for(TI i = 0; i < SPEC_1::SIZE; i++){
                target._data[i] = source._data[i];
            }
        }
        else{
            copy_view(source_device, target_device, source, target);
        }
    }
    template<typename DEV_SPEC, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT auto view(devices::CPU<DEV_SPEC>& device, const Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        using DEVICE = devices::CPU<DEV_SPEC>;
        static_assert(SPEC::ROWS >= ROWS);
        static_assert(SPEC::COLS >= COLS);
#ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_BOUNDS
        utils::assert_exit(device, (row + ROWS) <= SPEC::ROWS, "row + ROWS <= SPEC::ROWS");
        utils::assert_exit(device, (col + COLS) <= SPEC::COLS, "col + COLS <= SPEC::COLS");
#endif
        return _view<DEVICE, SPEC, ROWS, COLS>(device, m, row, col);
    }
    template<typename DEV_SPEC, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT std::vector<std::vector<typename SPEC::T>> std_vector(devices::CPU<DEV_SPEC>& device, Matrix<SPEC>& matrix){
        using DEVICE = devices::CPU<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        std::vector<std::vector<typename SPEC::T>> result;
        for(TI row_i = 0; row_i < SPEC::ROWS; row_i++){
            std::vector<typename SPEC::T> row;
            for(TI col_i = 0; col_i < SPEC::COLS; col_i++){
                row.push_back(get(matrix, row_i, col_i));
            }
            result.push_back(row);
        }
        return result;
    }
    template<typename DEVICE_SPEC, typename SPEC_A, typename SPEC_B, typename SPEC_C>
    void multiply_naive(devices::CPU<DEVICE_SPEC>& device, const Matrix<SPEC_A>& A, const Matrix<SPEC_B>& B, Matrix<SPEC_C>& C){
        static_assert(SPEC_A::ROWS == SPEC_C::ROWS);
        static_assert(SPEC_A::COLS == SPEC_B::ROWS);
        static_assert(SPEC_B::COLS == SPEC_C::COLS);
        using DEVICE = devices::CPU<DEVICE_SPEC>;

        using TA = typename SPEC_A::T;
        using TB = typename SPEC_B::T;
        using TC = typename SPEC_C::T;
        using TI = typename DEVICE::index_t;

        constexpr TI M = SPEC_A::ROWS;
        constexpr TI N = SPEC_B::COLS;
        constexpr TI K = SPEC_A::COLS;

        const TA * RL_TOOLS_RESTRICT A_data = A._data;
        const TB * RL_TOOLS_RESTRICT B_data = B._data;
        TC * RL_TOOLS_RESTRICT C_data = C._data;

        for (TI i = 0; i < M * N; ++i) {
            C_data[i] = (TC)0.0f;
        }

        for (TI i = 0; i < M; ++i) {
            for (TI k_idx = 0; k_idx < K; ++k_idx) {
                TA A_val = A_data[i * K + k_idx];
                for (TI j = 0; j < N; ++j) {
                    C_data[i * N + j] += A_val * B_data[k_idx * N + j];
                }
            }
        }
    }
    template<typename DEVICE_SPEC, typename SPEC_A, typename SPEC_B, typename SPEC_C>
    void multiply_tiled(devices::CPU<DEVICE_SPEC>& device, const Matrix<SPEC_A>& A, const Matrix<SPEC_B>& B, Matrix<SPEC_C>& C) {
        static_assert(SPEC_A::ROWS == SPEC_C::ROWS);
        static_assert(SPEC_A::COLS == SPEC_B::ROWS);
        static_assert(SPEC_B::COLS == SPEC_C::COLS);
        using DEVICE = devices::CPU<DEVICE_SPEC>;

        using TA = typename SPEC_A::T;
        using TB = typename SPEC_B::T;
        using TC = typename SPEC_C::T;
        using TI = typename DEVICE::index_t;

        constexpr TI M = SPEC_A::ROWS;
        constexpr TI N = SPEC_B::COLS;
        constexpr TI K = SPEC_A::COLS;
        constexpr TI blockSize = 32;

        const TA * RL_TOOLS_RESTRICT A_data = A._data;
        const TB * RL_TOOLS_RESTRICT B_data = B._data;
        TC * RL_TOOLS_RESTRICT C_data = C._data;

        // Initialize C to zero
        for (TI i = 0; i < M * N; ++i) {
            C_data[i] = (TC) 0.0f;
        }

        for (TI ii = 0; ii < M; ii += blockSize){
            for (TI kk = 0; kk < K; kk += blockSize){
                for (TI jj = 0; jj < N; jj += blockSize){
                    TI i_end = (ii + blockSize > M) ? M : ii + blockSize;
                    TI k_end = (kk + blockSize > K) ? K : kk + blockSize;
                    TI j_end = (jj + blockSize > N) ? N : jj + blockSize;
                    for (TI i = ii; i < i_end; ++i) {
                        for (TI k_idx = kk; k_idx < k_end; ++k_idx) {
                            TA A_val = A_data[i * K + k_idx];
                            for (TI j = jj; j < j_end; ++j) {
                                C_data[i * N + j] += A_val * B_data[k_idx * N + j];
                            }
                        }
                    }
                }
            }
        }
    }

    // template<typename DEVICE_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    // void multiply_tiled_simd(devices::CPU<DEVICE_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& C) {
    //     // Ensure matrix dimensions are compatible
    //     static_assert(INPUT_SPEC_A::ROWS == OUTPUT_SPEC::ROWS, "A.rows must equal C.rows");
    //     static_assert(INPUT_SPEC_A::COLS == INPUT_SPEC_B::ROWS, "A.cols must equal B.rows");
    //     static_assert(INPUT_SPEC_B::COLS == OUTPUT_SPEC::COLS, "B.cols must equal C.cols");
    //     using DEVICE = devices::CPU<DEVICE_SPEC>;
    //
    //     using T = typename OUTPUT_SPEC::T;
    //     using TI = typename DEVICE_SPEC::index_t; // Corrected index type
    //
    //     constexpr TI M = INPUT_SPEC_A::ROWS;
    //     constexpr TI N = INPUT_SPEC_B::COLS;
    //     constexpr TI K = INPUT_SPEC_A::COLS;
    //     constexpr TI blockSize = 32; // Adjust block size as needed
    //
    //     T* RL_TOOLS_RESTRICT A_data = A._data;
    //     T* RL_TOOLS_RESTRICT B_data = B._data;
    //     T* RL_TOOLS_RESTRICT C_data = C._data;
    //
    //     // Initialize C to zero
    //     for (TI i = 0; i < M * N; ++i) {
    //         C_data[i] = static_cast<T>(0);
    //     }
    //
    //     // Tiled matrix multiplication with SIMD intrinsics
    //     for (TI ii = 0; ii < M; ii += blockSize) {
    //         for (TI kk = 0; kk < K; kk += blockSize) {
    //             for (TI jj = 0; jj < N; jj += blockSize) {
    //                 TI i_end = (ii + blockSize > M) ? M : ii + blockSize;
    //                 TI k_end = (kk + blockSize > K) ? K : kk + blockSize;
    //                 TI j_end = (jj + blockSize > N) ? N : jj + blockSize;
    //
    //                 for (TI i = ii; i < i_end; ++i) {
    //                     for (TI k_idx = kk; k_idx < k_end; ++k_idx) {
    //                         T A_val = A_data[i * K + k_idx];
    //
    //                         // SIMD vectorization over the j loop
    //                         TI j = jj;
    //                         // Process 8 elements at a time using AVX
    //                         for (; j <= j_end - 8; j += 8) {
    //                             // Load B and C data into vectors
    //                             __m256 B_vec = _mm256_loadu_ps(&B_data[k_idx * N + j]);
    //                             __m256 C_vec = _mm256_loadu_ps(&C_data[i * N + j]);
    //                             __m256 A_vec = _mm256_set1_ps(A_val); // Broadcast A_val
    //
    //                             // Perform fused multiply-add: C_vec += A_vec * B_vec
    //                             C_vec = _mm256_fmadd_ps(A_vec, B_vec, C_vec);
    //
    //                             // Store the result back to C_data
    //                             _mm256_storeu_ps(&C_data[i * N + j], C_vec);
    //                         }
    //
    //                         // Handle remaining elements
    //                         for (; j < j_end; ++j) {
    //                             C_data[i * N + j] += A_val * B_data[k_idx * N + j];
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    // template<typename DEVICE_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    // void multiply_tiled_simd_ptr(devices::CPU<DEVICE_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& C) {
    //     // Ensure matrix dimensions are compatible
    //     static_assert(INPUT_SPEC_A::ROWS == OUTPUT_SPEC::ROWS, "A.rows must equal C.rows");
    //     static_assert(INPUT_SPEC_A::COLS == INPUT_SPEC_B::ROWS, "A.cols must equal B.rows");
    //     static_assert(INPUT_SPEC_B::COLS == OUTPUT_SPEC::COLS, "B.cols must equal C.cols");
    //
    //     using T = typename OUTPUT_SPEC::T;
    //     using TI = typename DEVICE_SPEC::index_t;
    //
    //     constexpr TI M = INPUT_SPEC_A::ROWS;
    //     constexpr TI N = INPUT_SPEC_B::COLS;
    //     constexpr TI K = INPUT_SPEC_A::COLS;
    //     constexpr TI blockSize = 32; // Adjust as needed
    //
    //     T* RL_TOOLS_RESTRICT A_data = A._data;
    //     T* RL_TOOLS_RESTRICT B_data = B._data;
    //     T* RL_TOOLS_RESTRICT C_data = C._data;
    //
    //     // Initialize C to zero
    //     std::fill(C_data, C_data + M * N, static_cast<T>(0));
    //
    //     // Tiled matrix multiplication with SIMD intrinsics and pointer arithmetic
    //     for (TI ii = 0; ii < M; ii += blockSize) {
    //         for (TI kk = 0; kk < K; kk += blockSize) {
    //             for (TI jj = 0; jj < N; jj += blockSize) {
    //                 TI i_end = std::min(ii + blockSize, M);
    //                 TI k_end = std::min(kk + blockSize, K);
    //                 TI j_end = std::min(jj + blockSize, N);
    //
    //                 for (TI i = ii; i < i_end; ++i) {
    //                     T* RL_TOOLS_RESTRICT C_row_ptr = C_data + i * N + jj;
    //                     T* RL_TOOLS_RESTRICT A_row_ptr = A_data + i * K + kk;
    //
    //                     for (TI k_idx = kk; k_idx < k_end; ++k_idx) {
    //                         T A_val = A_row_ptr[k_idx - kk]; // Offset within the block
    //                         T* RL_TOOLS_RESTRICT B_row_ptr = B_data + k_idx * N + jj;
    //
    //                         TI j = jj;
    //
    //                         // SIMD vectorization over the j loop
    //                         for (; j <= j_end - 8; j += 8) {
    //                             __m256 A_vec = _mm256_set1_ps(A_val);
    //                             __m256 B_vec = _mm256_loadu_ps(B_row_ptr + (j - jj));
    //                             __m256 C_vec = _mm256_loadu_ps(C_row_ptr + (j - jj));
    //
    //                             C_vec = _mm256_fmadd_ps(A_vec, B_vec, C_vec);
    //
    //                             _mm256_storeu_ps(C_row_ptr + (j - jj), C_vec);
    //                         }
    //
    //                         // Handle remaining elements
    //                         for (; j < j_end; ++j) {
    //                             C_row_ptr[j - jj] += A_val * B_row_ptr[j - jj];
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    template<typename DEVICE_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply(devices::CPU<DEVICE_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& C){
        using DEVICE = devices::CPU<DEVICE_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI M = INPUT_SPEC_A::ROWS;
        constexpr TI N = INPUT_SPEC_B::COLS;
        constexpr TI K = INPUT_SPEC_A::COLS;
        if constexpr(INPUT_SPEC_A::COL_PITCH == 1 && INPUT_SPEC_B::COL_PITCH == 1 && OUTPUT_SPEC::COL_PITCH == 1 && INPUT_SPEC_A::ROW_PITCH == INPUT_SPEC_A::COLS && INPUT_SPEC_B::ROW_PITCH == INPUT_SPEC_B::COLS && OUTPUT_SPEC::ROW_PITCH == OUTPUT_SPEC::COLS){
            if(M <= 32 || N <= 32 || K <= 32){
                multiply_tiled(device, A, B, C);
            }
            else{
                if(M <= 64 || N <= 64 || K <= 64){
                    multiply_tiled(device, A, B, C);
                } else {
                    multiply_naive(device, A, B, C);
                }
            }
        }
        else{
            multiply_generic<false>(device, A, B, C);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
