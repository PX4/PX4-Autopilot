#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_ESP32_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_ESP32_H

#include "operations_generic.h"
#include <cstring> // formemcpy
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SPEC_1, typename SPEC_2>
    void copy_view( const devices::ESP32<SOURCE_DEV_SPEC>& source_device, devices::ESP32<TARGET_DEV_SPEC>& target_device, Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        using SOURCE_DEVICE = devices::ESP32<SOURCE_DEV_SPEC>;
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using SPEC = SPEC_1;
        vectorize_unary<SOURCE_DEVICE, SPEC_1, SPEC_2, containers::vectorization::operators::copy<typename SOURCE_DEVICE::SPEC::MATH, typename SPEC::T>>(source_device, source, target);
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SPEC_1, typename SPEC_2>
    void copy( const devices::ESP32<SOURCE_DEV_SPEC>& source_device, devices::ESP32<TARGET_DEV_SPEC>& target_device, Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        if constexpr(containers::check_memory_layout<SPEC_1, SPEC_2>){
            std::memcpy(target._data, source._data, SPEC_1::SIZE_BYTES);
        }
        else{
            copy_view(source_device, target_device, source, target);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
