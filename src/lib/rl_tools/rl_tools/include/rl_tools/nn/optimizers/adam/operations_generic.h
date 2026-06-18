#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_OPTIMIZERS_ADAM_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_OPTIMIZERS_ADAM_OPERATIONS_GENERIC_H

#include "adam.h"
#include "../../../nn/layers/dense/layer.h"
#include "../../../nn/parameters/operations_generic.h"
#include "../../../utils/polyak/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer){
        malloc(device, optimizer.age);
        malloc(device, optimizer.first_order_moment_bias_correction);
        malloc(device, optimizer.second_order_moment_bias_correction);
        malloc(device, optimizer.parameters);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer){
        free(device, optimizer.age);
        free(device, optimizer.first_order_moment_bias_correction);
        free(device, optimizer.second_order_moment_bias_correction);
        free(device, optimizer.parameters);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer){
        typename nn::optimizers::Adam<SPEC>::PARAMETERS parameters = {
            SPEC::DEFAULT_PARAMETERS::ALPHA,
            SPEC::DEFAULT_PARAMETERS::BETA_1,
            SPEC::DEFAULT_PARAMETERS::BETA_2,
            SPEC::DEFAULT_PARAMETERS::EPSILON,
            SPEC::DEFAULT_PARAMETERS::EPSILON_SQRT,
            SPEC::DEFAULT_PARAMETERS::WEIGHT_DECAY,
            SPEC::DEFAULT_PARAMETERS::WEIGHT_DECAY_INPUT,
            SPEC::DEFAULT_PARAMETERS::WEIGHT_DECAY_OUTPUT,
            SPEC::DEFAULT_PARAMETERS::BIAS_LR_FACTOR
        };
        set(device, optimizer.parameters, parameters, 0);
    }
    template<typename DEVICE, typename SPEC, typename MODEL>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_optimizer_state(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer, MODEL& model) {
        set(device, optimizer.age, 1, 0);
        _reset_optimizer_state(device, model, optimizer);
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void _step(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        TI age = get(device, optimizer.age, 0);
        const auto& optimizer_parameters = get(device, optimizer.parameters, 0);
        T first_order_moment_bias_correction  = 1/(1 - math::pow(device.math, optimizer_parameters.beta_1, (T)age));
        set(device, optimizer.first_order_moment_bias_correction, first_order_moment_bias_correction, 0);
        T second_order_moment_bias_correction = 1/(1 - math::pow(device.math, optimizer_parameters.beta_2, (T)age));
        set(device, optimizer.second_order_moment_bias_correction, second_order_moment_bias_correction, 0);
        set(device, optimizer.age, age + 1, 0);
    }
    template<typename DEVICE, typename SPEC, typename MODEL>
    RL_TOOLS_FUNCTION_PLACEMENT void step(DEVICE& device, nn::optimizers::Adam<SPEC>& optimizer, MODEL& model) {
        _step(device, optimizer);
        update(device, model, optimizer);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::optimizers::Adam<SOURCE_SPEC>& source, nn::optimizers::Adam<TARGET_SPEC>& target){
        copy(source_device, target_device, source.age, target.age);
        copy(source_device, target_device, source.parameters, target.parameters);
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const nn::parameters::Adam::Instance<SPEC>& p){
        bool param_nan = is_nan(device, (nn::parameters::Gradient::Instance<SPEC>&) p);
        return param_nan || is_nan(device, p.gradient_first_order_moment) || is_nan(device, p.gradient_second_order_moment);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
