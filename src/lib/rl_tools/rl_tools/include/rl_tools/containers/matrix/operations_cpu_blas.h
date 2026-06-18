#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_BLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_BLAS_H

#include "operations_cpu.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace containers::matrix{
        template<bool ACCUMULATE, typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
        void multiply_blas(devices::CPU_BLAS<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output) {
            static_assert(INPUT_SPEC_A::ROWS == OUTPUT_SPEC::ROWS);
            static_assert(INPUT_SPEC_A::COLS == INPUT_SPEC_B::ROWS);
            static_assert(INPUT_SPEC_B::COLS == OUTPUT_SPEC::COLS);
            static_assert(INPUT_SPEC_A::ROW_PITCH == 1 || INPUT_SPEC_A::COL_PITCH == 1); // dense row- or column-major
            static_assert(INPUT_SPEC_B::ROW_PITCH == 1 || INPUT_SPEC_B::COL_PITCH == 1); // dense row- or column-major

            using T = typename OUTPUT_SPEC::T;
            using TI = typename DEV_SPEC::index_t;

            constexpr bool A_ROW_MAJOR = INPUT_SPEC_A::ROW_PITCH >= INPUT_SPEC_A::COLS;
            constexpr bool B_ROW_MAJOR = INPUT_SPEC_B::ROW_PITCH >= INPUT_SPEC_B::COLS;
            constexpr auto A_TRANSPOSE = A_ROW_MAJOR ? CblasNoTrans : CblasTrans;
            constexpr auto B_TRANSPOSE = B_ROW_MAJOR ? CblasNoTrans : CblasTrans;

            constexpr TI A_PITCH = A_ROW_MAJOR ? INPUT_SPEC_A::ROW_PITCH : INPUT_SPEC_A::COL_PITCH;
            constexpr TI B_PITCH = B_ROW_MAJOR ? INPUT_SPEC_B::ROW_PITCH : INPUT_SPEC_B::COL_PITCH;

            constexpr T alpha = 1;
            constexpr T beta = ACCUMULATE ? 1 : 0;
            constexpr auto m = OUTPUT_SPEC::ROWS;
            constexpr auto k = INPUT_SPEC_A::COLS;
            constexpr auto n = OUTPUT_SPEC::COLS;

            if constexpr(utils::typing::is_same_v<T, float>){
                cblas_sgemm(CblasRowMajor, A_TRANSPOSE, B_TRANSPOSE, m, n, k, alpha, A._data, A_PITCH, B._data, B_PITCH, beta, output._data, row_pitch(output));
            }
            else{
                cblas_dgemm(CblasRowMajor, A_TRANSPOSE, B_TRANSPOSE, m, n, k, alpha, A._data, A_PITCH, B._data, B_PITCH, beta, output._data, row_pitch(output));
            }
        }
    }
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply(devices::CPU_BLAS<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        containers::matrix::multiply_blas<false>(device, A, B, output);
    }
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply_accumulate(devices::CPU_BLAS<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        containers::matrix::multiply_blas<true>(device, A, B, output);
    }
    template<typename SOURCE_DEVICE_SPEC, typename TARGET_DEVICE_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CPU_BLAS<SOURCE_DEVICE_SPEC>& source_device, devices::CPU_BLAS<TARGET_DEVICE_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(static_cast<devices::CPU<SOURCE_DEVICE_SPEC>&>(source_device), static_cast<devices::CPU<TARGET_DEVICE_SPEC>&>(target_device), source, target);
    }
    template<typename SOURCE_DEVICE_SPEC, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CPU_BLAS<SOURCE_DEVICE_SPEC>& source_device, TARGET_DEVICE& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(static_cast<devices::CPU<SOURCE_DEVICE_SPEC>&>(source_device), target_device, source, target);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(SOURCE_DEVICE& source_device, devices::CPU_BLAS<TARGET_DEVICE_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(source_device, static_cast<devices::CPU<TARGET_DEVICE_SPEC>&>(target_device), source, target);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif