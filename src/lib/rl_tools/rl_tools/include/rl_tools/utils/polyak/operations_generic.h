#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_UTILS_POLYAK_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_UTILS_POLYAK_OPERATIONS_GENERIC_H


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils::polyak {
    // todo: polyak factor as template parameter (reciprocal INT e.g.)
    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, typename T_POLYAK>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, const  Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target, const T_POLYAK polyak, bool clip = false, typename SOURCE_SPEC::T clip_value = 1){
        static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
        using SPEC = SOURCE_SPEC;
        using T = typename SPEC::T;
        for(typename DEVICE::index_t i = 0; i < SPEC::ROWS; i++) {
            for(typename DEVICE::index_t j = 0; j < SPEC::COLS; j++) {
                T source_value = get(source, i, j);
                if(clip){
                    source_value = math::clamp(device.math, source_value, -clip_value, clip_value);
                }
                set(target, i, j, polyak * get(target, i, j) + (1 - polyak) * source_value);
            }
        }
    }

    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, typename T_POLYAK>
    RL_TOOLS_FUNCTION_PLACEMENT void update_squared(DEVICE& device, const  Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target, const T_POLYAK polyak, bool clip = false, typename SOURCE_SPEC::T clip_value = 1) {
        static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
        using SPEC = SOURCE_SPEC;
        using T = typename SPEC::T;
        for(typename DEVICE::index_t i = 0; i < SPEC::ROWS; i++) {
            for(typename DEVICE::index_t j = 0; j < SPEC::COLS; j++) {
                T source_value = get(source, i, j);
                if(clip){
                    source_value = math::clamp(device.math, source_value, -clip_value, clip_value);
                }
                set(target, i, j, polyak * get(target, i, j) + (1 - polyak) * source_value * source_value);
            }
        }
    }
    namespace binary_kernels{
        template <typename T>
        struct PolyakParameters{
            T polyak;
            bool clip;
            T clip_value;
        };
        template <typename T>
        struct PolyakUpdate {
            PolyakParameters<T> parameters;
            template <typename DEVICE, typename T_T>
            RL_TOOLS_FUNCTION_PLACEMENT static T_T operation(DEVICE& device, const PolyakUpdate<T_T>& p, T_T source, T_T target){
                if(p.parameters.clip) {
                    source = source > p.parameters.clip_value ? p.parameters.clip_value : (source < -p.parameters.clip_value ? -p.parameters.clip_value : source);
                }
                return p.parameters.polyak * target + (1 - p.parameters.polyak) * source;
            }
        };
        template <typename T>
        struct PolyakUpdateSquared {
            PolyakParameters<T> parameters;
            template <typename DEVICE, typename T_T>
            RL_TOOLS_FUNCTION_PLACEMENT static T_T operation(DEVICE& device, const PolyakUpdateSquared<T_T>& p, T_T source, T_T target){
                if(p.parameters.clip) {
                    source = source > p.parameters.clip_value ? p.parameters.clip_value : (source < -p.parameters.clip_value ? -p.parameters.clip_value : source);
                }
                return p.parameters.polyak * target + (1 - p.parameters.polyak) * source * source;
            }
        };
    }
    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, typename T_POLYAK>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, const  Tensor<SOURCE_SPEC>& source, Tensor<TARGET_SPEC>& target, const T_POLYAK polyak, const bool clip = false, typename SOURCE_SPEC::T clip_value = 1) {
        binary_kernels::PolyakUpdate<T_POLYAK> params{};
        params.parameters.polyak = polyak;
        params.parameters.clip = clip;
        params.parameters.clip_value = clip_value;
        binary_operation(device, params, source, target);
    }

    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, typename T_POLYAK>
    RL_TOOLS_FUNCTION_PLACEMENT void update_squared(DEVICE& device, const  Tensor<SOURCE_SPEC>& source, Tensor<TARGET_SPEC>& target, const T_POLYAK polyak, const bool clip = false, typename SOURCE_SPEC::T clip_value = 1) {
        using T = typename SOURCE_SPEC::T;
        binary_kernels::PolyakUpdateSquared<T> params{};
        params.parameters.polyak = polyak;
        params.parameters.clip = clip;
        params.parameters.clip_value = clip_value;
        binary_operation(device, params, source, target);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif