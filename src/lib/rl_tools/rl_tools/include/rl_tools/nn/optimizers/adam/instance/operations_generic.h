#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_OPTIMIZERS_ADAM_INSTANCE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_OPTIMIZERS_ADAM_INSTANCE_OPERATIONS_GENERIC_H

#include "../adam.h"
#include "../../../../nn/layers/dense/layer.h"
#include "../../../../nn/parameters/operations_generic.h"
#include "../../../../utils/polyak/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::parameters::Adam::Instance<SPEC>& p){
        malloc(device, (nn::parameters::Gradient::Instance<SPEC>&) p);
        malloc(device, p.gradient_first_order_moment);
        malloc(device, p.gradient_second_order_moment);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::parameters::Adam::Instance<SPEC>& p){
        free(device, (nn::parameters::Gradient::Instance<SPEC>&) p);
        free(device, p.gradient_first_order_moment);
        free(device, p.gradient_second_order_moment);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void gradient_descent(DEVICE& device, nn::parameters::Adam::Instance<PARAMETER_SPEC>& parameter, nn::optimizers::Adam<SPEC>& optimizer){
        using TI = typename DEVICE::index_t;
        using T_OPTIMIZER = typename PARAMETER_SPEC::TYPE_POLICY::template GET<numeric_types::categories::OptimizerState>;
        using T_PARAMETER = typename decltype(parameter.parameters)::T;
        const auto& optimizer_parameters = get(device, optimizer.parameters, 0);
        auto parameters = matrix_view(device, parameter.parameters);
        auto gradient_first_order_moment = matrix_view(device, parameter.gradient_first_order_moment);
        auto gradient_second_order_moment = matrix_view(device, parameter.gradient_second_order_moment);
        constexpr TI ROWS = decltype(parameters)::ROWS;
        constexpr TI COLS = decltype(parameters)::COLS;
        for(TI row_i = 0; row_i < ROWS; row_i++) {
            for(TI col_i = 0; col_i < COLS; col_i++) {
                T_OPTIMIZER pre_sqrt_term = get(gradient_second_order_moment, row_i, col_i) * get(device, optimizer.second_order_moment_bias_correction, 0);
                pre_sqrt_term = math::max(device.math, pre_sqrt_term, (T_OPTIMIZER)optimizer_parameters.epsilon_sqrt);
                T_OPTIMIZER parameter_update = optimizer_parameters.alpha * get(device, optimizer.first_order_moment_bias_correction, 0) * get(gradient_first_order_moment, row_i, col_i) / (math::sqrt(device.math, pre_sqrt_term) + optimizer_parameters.epsilon);
                if constexpr(utils::typing::is_same_v<typename PARAMETER_SPEC::CATEGORY_TAG, nn::parameters::categories::Biases> && SPEC::ENABLE_BIAS_LR_FACTOR){
                    parameter_update *= optimizer_parameters.bias_lr_factor;
                }
                if constexpr(utils::typing::is_same_v<typename PARAMETER_SPEC::CATEGORY_TAG, nn::parameters::categories::Weights>){
                    if constexpr(utils::typing::is_same_v<typename PARAMETER_SPEC::GROUP_TAG, nn::parameters::groups::Normal> && SPEC::ENABLE_WEIGHT_DECAY){
                        parameter_update += get(parameters, row_i, col_i) * optimizer_parameters.weight_decay / 2;
                    }
                    if constexpr(utils::typing::is_same_v<typename PARAMETER_SPEC::GROUP_TAG, nn::parameters::groups::Input> && SPEC::ENABLE_WEIGHT_DECAY){
                        parameter_update += get(parameters, row_i, col_i) * optimizer_parameters.weight_decay_input / 2;
                    }
                    if constexpr(utils::typing::is_same_v<typename PARAMETER_SPEC::GROUP_TAG, nn::parameters::groups::Output> && SPEC::ENABLE_WEIGHT_DECAY){
                        parameter_update += get(parameters, row_i, col_i) * optimizer_parameters.weight_decay_output / 2;
                    }
                }
                T_OPTIMIZER value = get(parameters, row_i, col_i);
                value -= parameter_update;
                set(parameters, row_i, col_i, (T_PARAMETER)value);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename ADAM_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::parameters::Adam::Instance<SPEC>& parameter, nn::optimizers::Adam<ADAM_SPEC>& optimizer) {
        using PARAMETERS = typename ADAM_SPEC::DEFAULT_PARAMETERS;
        const auto& optimizer_parameters = get(device, optimizer.parameters, 0);
        utils::polyak::update(device, parameter.gradient, parameter.gradient_first_order_moment, optimizer_parameters.beta_1, PARAMETERS::ENABLE_GRADIENT_CLIPPING, PARAMETERS::GRADIENT_CLIP_VALUE);
        utils::polyak::update_squared(device, parameter.gradient, parameter.gradient_second_order_moment, optimizer_parameters.beta_2, PARAMETERS::ENABLE_GRADIENT_CLIPPING, PARAMETERS::GRADIENT_CLIP_VALUE);
        gradient_descent(device, parameter, optimizer);
    }


    template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::parameters::Adam::Instance<PARAMETER_SPEC>& parameter, nn::optimizers::Adam<SPEC>& optimizer){
        set_all(device, parameter.gradient_first_order_moment, 0);
        set_all(device, parameter.gradient_second_order_moment, 0);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::parameters::Adam::Instance<SOURCE_SPEC>& source, nn::parameters::Adam::Instance<TARGET_SPEC>& target){
        copy(source_device, target_device, (nn::parameters::Gradient::Instance<SOURCE_SPEC>&) source, (nn::parameters::Gradient::Instance<TARGET_SPEC>&) target);
        copy(source_device, target_device, source.gradient_first_order_moment , target.gradient_first_order_moment);
        copy(source_device, target_device, source.gradient_second_order_moment, target.gradient_second_order_moment);
    }
    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::CONTAINER::T abs_diff(DEVICE& device, const nn::parameters::Adam::Instance<SPEC_1>& p1, const nn::parameters::Adam::Instance<SPEC_2>& p2){
        typename SPEC_1::CONTAINER::T acc = 0;
        acc += abs_diff(device, static_cast<const nn::parameters::Gradient::Instance<SPEC_1>&>(p1), static_cast<const nn::parameters::Gradient::Instance<SPEC_2>&>(p2));
        acc += abs_diff(device, p1.gradient_first_order_moment, p2.gradient_first_order_moment);
        acc += abs_diff(device, p1.gradient_second_order_moment, p2.gradient_second_order_moment);
        return acc;
    }
    template<typename DEVICE, typename SPEC, typename MODE = Mode<mode::Default<>>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const nn::parameters::Adam::Instance<SPEC>& p, const Mode<MODE>& mode = {}){
        bool upstream_nan = is_nan(device, static_cast<const nn::parameters::Gradient::Instance<SPEC>&>(p), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, p.gradient_first_order_moment, mode) || is_nan(device, p.gradient_second_order_moment, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif