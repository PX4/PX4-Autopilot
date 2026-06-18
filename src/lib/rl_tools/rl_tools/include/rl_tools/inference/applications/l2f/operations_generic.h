#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_INFERENCE_APPLICATIONS_L2F_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_INFERENCE_APPLICATIONS_L2F_OPERATIONS_GENERIC_H

#include "l2f.h"
#include "../../executor/operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, inference::applications::L2F<SPEC>& executor){
        malloc(device, executor.input);
        malloc(device, executor.output);
        malloc(device, executor.executor);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, inference::applications::L2F<SPEC>& executor){
        free(device, executor.input);
        free(device, executor.output);
        free(device, executor.executor);
    }
    template <typename DEVICE, typename SPEC, typename POLICY, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, inference::applications::L2F<SPEC>& executor, POLICY& policy, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr T HOVERING_THROTTLE = 0.66;
        for(TI step_i = 0; step_i < SPEC::ACTION_HISTORY_LENGTH; step_i++){
            for(TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                executor.action_history[step_i][action_i] = HOVERING_THROTTLE * 2 - 1;
            }
        }
        executor.steps_since_original_control_step = 0;
        reset(device, executor.executor, policy, rng);
    }
    namespace inference::applications::l2f{
        template <typename DEVICE, typename SPEC, typename OBS_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void observe(DEVICE& device, L2F<SPEC>& executor, Observation<SPEC>& observation, Tensor<OBS_SPEC>& observation_flat){
            using TI = typename DEVICE::index_t;
            static_assert(OBS_SPEC::SHAPE::template GET<0> == 1);
            static_assert(OBS_SPEC::SHAPE::template GET<1> == 18 + SPEC::OUTPUT_DIM * SPEC::ACTION_HISTORY_LENGTH); // position + orientation + linear_velocity + angular_velocity + action_history
            TI base = 0;
            set(device, observation_flat, observation.position[0], 0,  base++);
            set(device, observation_flat, observation.position[1], 0,  base++);
            set(device, observation_flat, observation.position[2], 0,  base++);
            float qw = observation.orientation[0];
            float qx = observation.orientation[1];
            float qy = observation.orientation[2];
            float qz = observation.orientation[3];
            set(device, observation_flat,   (1 - 2*qy*qy - 2*qz*qz), 0, base++);
            set(device, observation_flat,   (    2*qx*qy - 2*qw*qz), 0, base++);
            set(device, observation_flat,   (    2*qx*qz + 2*qw*qy), 0, base++);
            set(device, observation_flat,   (    2*qx*qy + 2*qw*qz), 0, base++);
            set(device, observation_flat,   (1 - 2*qx*qx - 2*qz*qz), 0, base++);
            set(device, observation_flat,   (    2*qy*qz - 2*qw*qx), 0, base++);
            set(device, observation_flat,   (    2*qx*qz - 2*qw*qy), 0, base++);
            set(device, observation_flat,   (    2*qy*qz + 2*qw*qx), 0, base++);
            set(device, observation_flat,   (1 - 2*qx*qx - 2*qy*qy), 0, base++);
            set(device, observation_flat, observation.linear_velocity[0], 0, base++);
            set(device, observation_flat, observation.linear_velocity[1], 0, base++);
            set(device, observation_flat, observation.linear_velocity[2], 0, base++);
            set(device, observation_flat, observation.angular_velocity[0], 0, base++);
            set(device, observation_flat, observation.angular_velocity[1], 0, base++);
            set(device, observation_flat, observation.angular_velocity[2], 0, base++);
            for(TI step_i = 0; step_i < SPEC::ACTION_HISTORY_LENGTH; step_i++){
                for(TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                    set(device, observation_flat, executor.action_history[step_i][action_i], 0, base++);
                }
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename POLICY, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT auto control(DEVICE& device, inference::applications::L2F<SPEC>& executor, typename SPEC::TIMESTAMP nanoseconds, POLICY& policy, inference::applications::l2f::Observation<SPEC>& observation, inference::applications::l2f::Action<SPEC>& action, RNG& rng){
        using TI = typename SPEC::TI;
        if(executor.steps_since_original_control_step == 0){
            for(TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                executor.action_history[0][action_i] = observation.previous_action[action_i];
            }
        }
        else{
            for(TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                executor.action_history[0][action_i] = (executor.action_history[0][action_i] * executor.steps_since_original_control_step + observation.previous_action[action_i]) / (executor.steps_since_original_control_step + 1);
            }
        }
        inference::applications::l2f::observe(device, executor, observation, executor.input);
        auto status = control(device, executor.executor, nanoseconds, policy, executor.input, executor.output, rng);
        for (TI output_i=0; output_i < SPEC::OUTPUT_DIM; output_i++){
            action.action[output_i] = get(device, executor.output, 0, output_i);
        }

        executor.steps_since_original_control_step++; // gets overwritten with 0 in the case of an original control step
        if(status.source == decltype(status.source)::CONTROL){
            if(status.step_type == decltype(status.step_type)::NATIVE){
                // step action history
                static_assert(SPEC::ACTION_HISTORY_LENGTH >= 1);
                for(TI step_i = SPEC::ACTION_HISTORY_LENGTH-1; step_i > 0; step_i--){
                    for(TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                        executor.action_history[step_i][action_i] = executor.action_history[step_i-1][action_i];
                    }
                }
                executor.steps_since_original_control_step = 0;
            }
        }
        return status;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
