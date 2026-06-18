
#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_ACCELERATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_ACCELERATE_H

#include "tensor.h"
#include "operations_cpu.h"
#include "operations_cpu_blas.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    void matrix_multiply(devices::CPU_ACCELERATE<DEV_SPEC>& device, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
        matrix_multiply(static_cast<devices::CPU_BLAS<DEV_SPEC>&>(device), t1, t2, result);
    }
    template<typename DEV_SPEC, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    void matrix_multiply_accumulate(devices::CPU_ACCELERATE<DEV_SPEC>& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
        matrix_multiply_accumulate(static_cast<devices::CPU_BLAS<DEV_SPEC>&>(device), t1, t2, result);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
