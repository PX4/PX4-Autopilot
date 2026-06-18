#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_PARAMETERS_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_PARAMETERS_OPERATIONS_GENERIC_H

#include "parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename CONTAINER>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::parameters::Plain::Instance<CONTAINER>& p){
        malloc(device, p.parameters);
    }
    template <typename DEVICE, typename CONTAINER>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::parameters::Plain::Instance<CONTAINER>& p){
        free(device, p.parameters);
    }
    template <typename DEVICE, typename CONTAINER>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::parameters::Gradient::Instance<CONTAINER>& p){
        malloc(device, (nn::parameters::Plain::Instance<CONTAINER>&) p);
        malloc(device, p.gradient);
    }
    template <typename DEVICE, typename CONTAINER>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::parameters::Gradient::Instance<CONTAINER>& p){
        free(device, (nn::parameters::Plain::Instance<CONTAINER>&) p);
        free(device, p.gradient);
    }
    template<typename DEVICE, typename CONTAINER>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::parameters::Gradient::Instance<CONTAINER>& container) {
        set_all(device, container.gradient, 0);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::parameters::Plain::Instance<SOURCE_SPEC>& source, nn::parameters::Plain::Instance<TARGET_SPEC>& target){
        copy(source_device, target_device, source.parameters, target.parameters);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::parameters::Gradient::Instance<SOURCE_SPEC>& source, nn::parameters::Gradient::Instance<TARGET_SPEC>& target){
        copy(source_device, target_device, (nn::parameters::Plain::Instance<SOURCE_SPEC>&) source, (nn::parameters::Plain::Instance<TARGET_SPEC>&) target);
        copy(source_device, target_device, source.gradient, target.gradient);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::parameters::Plain::Instance<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, source.parameters, target.parameters);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::parameters::Gradient::Instance<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, static_cast<typename SOURCE::PARENT&>(source), static_cast<nn::parameters::Plain::Instance<TARGET_SPEC>&>(target));
        copy_from_generic(source_device, target_device, source.gradient, target.gradient);
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT auto abs_diff(DEVICE& device, const nn::parameters::Plain::Instance<SPEC_1>& p1, const nn::parameters::Plain::Instance<SPEC_2>& p2){
        typename decltype(p1.parameters)::SPEC::T acc = 0;
        acc += abs_diff(device, p1.parameters, p2.parameters);
        return acc;
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT auto abs_diff(DEVICE& device, const nn::parameters::Gradient::Instance<SPEC_1>& p1, const nn::parameters::Gradient::Instance<SPEC_2>& p2){
        typename decltype(p1.parameters)::SPEC::T acc = 0;
        acc += abs_diff(device, (nn::parameters::Plain::Instance<SPEC_1>&) p1, (nn::parameters::Plain::Instance<SPEC_2>&) p2);
        acc += abs_diff(device, p1.gradient, p2.gradient);
        return acc;
    }

    template<typename DEVICE, typename SPEC, typename MODE = Mode<mode::Default<>>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const nn::parameters::Plain::Instance<SPEC>& p, const Mode<MODE>& mode = {}){
        return is_nan(device, p.parameters, mode);
    }


    template<typename DEVICE, typename SPEC, typename MODE = Mode<mode::Default<>>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const nn::parameters::Gradient::Instance<SPEC>& p, const Mode<MODE>& mode = {}){
        bool upstream_nan = is_nan(device, static_cast<const nn::parameters::Plain::Instance<SPEC>&>(p), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, p.gradient, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::parameters::Gradient::Instance<SPEC>& parameter){
        return squared_sum(device, parameter.gradient);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
