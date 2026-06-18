#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_OPENBLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CPU_OPENBLAS_H

#include "operations_cpu_blas.h"
#include "matrix.h"
#include "../../devices/cpu_openblas.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply(devices::CPU_OPENBLAS<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output) {
        multiply(static_cast<devices::CPU_BLAS<DEV_SPEC>&>(device), A, B, output);
    }
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply_accumulate(devices::CPU_OPENBLAS<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output) {
        multiply_accumulate(static_cast<devices::CPU_BLAS<DEV_SPEC>&>(device), A, B, output);
    }
    template<typename SOURCE_DEVICE_SPEC, typename TARGET_DEVICE_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CPU_OPENBLAS<SOURCE_DEVICE_SPEC>& source_device, devices::CPU_OPENBLAS<TARGET_DEVICE_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(static_cast<devices::CPU_BLAS<SOURCE_DEVICE_SPEC>&>(source_device), static_cast<devices::CPU_BLAS<TARGET_DEVICE_SPEC>&>(target_device), source, target);
    }
    template<typename SOURCE_DEVICE_SPEC, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CPU_OPENBLAS<SOURCE_DEVICE_SPEC>& source_device, TARGET_DEVICE& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(static_cast<devices::CPU_BLAS<SOURCE_DEVICE_SPEC>&>(source_device), target_device, source, target);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(SOURCE_DEVICE& source_device, devices::CPU_OPENBLAS<TARGET_DEVICE_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        copy(source_device, static_cast<devices::CPU_BLAS<TARGET_DEVICE_SPEC>&>(target_device), source, target);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
