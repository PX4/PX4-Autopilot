#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_DUMMY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_DUMMY_H

#include "operations_generic.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(devices::Dummy<SOURCE_DEV_SPEC>& source_device, devices::Dummy<TARGET_DEV_SPEC>& target_device, const Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        using SOURCE_DEVICE = devices::Dummy<SOURCE_DEV_SPEC>;
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using SPEC = SPEC_1;
        vectorize_unary<SOURCE_DEVICE, SPEC_1, SPEC_2, containers::vectorization::operators::copy<typename SOURCE_DEVICE::SPEC::MATH, typename SPEC::T>>(source_device, source, target);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
