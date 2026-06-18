#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_INFERENCE_EXECUTOR_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_INFERENCE_EXECUTOR_OPERATIONS_GENERIC_H

#include "executor.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, inference::Executor<SPEC>& executor){
        malloc(device, executor.observation);
        malloc(device, executor.policy_state);
        malloc(device, executor.policy_state_temp);
        malloc(device, executor.policy_buffer);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, inference::Executor<SPEC>& executor){
        free(device, executor.observation);
        free(device, executor.policy_state);
        free(device, executor.policy_state_temp);
        free(device, executor.policy_buffer);
    }
    template <typename DEVICE, typename SPEC, typename POLICY, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, inference::Executor<SPEC>& executor, POLICY& policy, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        reset(device, policy, executor.policy_state, rng);
        executor.last_observation_timestamp_set = false;
        executor.last_control_timestamp_set = false;
        executor.last_control_timestamp_original_set = false;
        executor.control_dt_index = 0;
        executor.control_original_dt_index = 0;
        executor.intermediate_step = 0;
        if(!executor.force_sync_native_initialized){
            executor.force_sync_native = SPEC::FORCE_SYNC_NATIVE;
        }
        executor.force_sync_native_initialized = true;
    }

    namespace inference::executor{
        template <bool ORIGINAL, typename DEVICE, typename SPEC>
        JitterStatus<SPEC> timing_jitter_status(DEVICE& device, Executor<SPEC>& executor){
            using T = typename SPEC::T;
            using TI = typename SPEC::TI;
            JitterStatus<SPEC> result;
            result.OK = true;
            result.MAGNITUDE = 0;
            if((ORIGINAL ? executor.control_original_dt_index : executor.control_dt_index) < SPEC::TIMING_STATS_NUM_STEPS){
                return result;
            }
            for(TI i = 0; i < SPEC::TIMING_STATS_NUM_STEPS; i++){
                auto intermediate_interval = SPEC::FORCE_SYNC_NATIVE_RUNTIME && executor.force_sync_native != 0 ? SPEC::CONTROL_INTERVAL_NATIVE_NS / executor.force_sync_native : SPEC::CONTROL_INTERVAL_INTERMEDIATE_NS;
                auto value = ORIGINAL ? executor.control_original_dt[i] : executor.control_dt[i];
                auto expected = ORIGINAL ? SPEC::CONTROL_INTERVAL_NATIVE_NS : intermediate_interval;
                if(value > expected * (ORIGINAL ? SPEC::WARNING_LEVELS::NATIVE_TIMING_JITTER_HIGH_THRESHOLD : SPEC::WARNING_LEVELS::INTERMEDIATE_TIMING_JITTER_HIGH_THRESHOLD) || value < expected * (ORIGINAL ? SPEC::WARNING_LEVELS::NATIVE_TIMING_JITTER_LOW_THRESHOLD : SPEC::WARNING_LEVELS::INTERMEDIATE_TIMING_JITTER_LOW_THRESHOLD)){
                    T magnitude = value / (T)expected;
                    result.OK = false;
                    result.MAGNITUDE = magnitude;
                    return result;
                }
            }
            return result;
        }

        template <bool ORIGINAL, typename DEVICE, typename SPEC>
        BiasStatus<SPEC> timing_bias_status(DEVICE& device, Executor<SPEC>& executor){
            using T = typename SPEC::T;
            using TI = typename SPEC::TI;
            BiasStatus<SPEC> result;
            result.OK = true;
            result.MAGNITUDE = 0;
            if((ORIGINAL ? executor.control_original_dt_index : executor.control_dt_index) < SPEC::TIMING_STATS_NUM_STEPS){
                return result;
            }
            T bias = 0;
            for(TI i = 0; i < SPEC::TIMING_STATS_NUM_STEPS; i++){
                bias += ORIGINAL ? executor.control_original_dt[i] : executor.control_dt[i];
            }
            bias /= SPEC::TIMING_STATS_NUM_STEPS;

            auto intermediate_interval = SPEC::FORCE_SYNC_NATIVE_RUNTIME && executor.force_sync_native != 0 ? SPEC::CONTROL_INTERVAL_NATIVE_NS / executor.force_sync_native : SPEC::CONTROL_INTERVAL_INTERMEDIATE_NS;
            auto expected = ORIGINAL ? SPEC::CONTROL_INTERVAL_NATIVE_NS : intermediate_interval;
            T magnitude = bias / (T)expected;
            result.MAGNITUDE = magnitude;
            if(bias > expected * (ORIGINAL ? SPEC::WARNING_LEVELS::NATIVE_TIMING_BIAS_HIGH_THRESHOLD : SPEC::WARNING_LEVELS::INTERMEDIATE_TIMING_BIAS_HIGH_THRESHOLD) || bias < expected * (ORIGINAL ? SPEC::WARNING_LEVELS::NATIVE_TIMING_BIAS_LOW_THRESHOLD : SPEC::WARNING_LEVELS::INTERMEDIATE_TIMING_BIAS_LOW_THRESHOLD)){
                result.OK = false;
            }
            return result;
        }
    }

    template <typename DEVICE, typename SPEC, typename POLICY, typename OBS_SPEC, typename ACTION_SPEC, typename RNG>
    inference::executor::Status<SPEC> control(DEVICE&device, inference::Executor<SPEC>& executor, typename SPEC::TIMESTAMP nanoseconds, POLICY& policy, Tensor<OBS_SPEC>& observation, Tensor<ACTION_SPEC>& action, RNG& rng){
        static_assert(OBS_SPEC::SHAPE::LENGTH == 2);
        static_assert(OBS_SPEC::SHAPE::FIRST == 1);
        static_assert(OBS_SPEC::SHAPE::LAST == SPEC::INPUT_DIM);
        static_assert(ACTION_SPEC::SHAPE::LENGTH == 2);
        static_assert(ACTION_SPEC::SHAPE::FIRST == 1);
        static_assert(ACTION_SPEC::SHAPE::LAST == SPEC::OUTPUT_DIM);
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using TIMESTAMP = typename SPEC::TIMESTAMP;
        bool reset = false;
        if(!executor.last_observation_timestamp_set){
            executor.last_observation_timestamp = nanoseconds;
            executor.last_observation_timestamp_set = true;
        }
        if(!executor.last_control_timestamp_set){
            reset = true;
            executor.last_control_timestamp = nanoseconds;
            executor.last_control_timestamp_set = true;
        }
        if(nanoseconds < executor.last_observation_timestamp){
            executor.last_observation_timestamp = nanoseconds;
            executor.last_observation_timestamp_set = true;
            inference::executor::Status<SPEC> status = {};
            status.OK = false;
            status.TIMESTAMP_INVALID = true;
            return status;
        }
        if(nanoseconds < executor.last_control_timestamp){
            executor.last_control_timestamp = nanoseconds;
            executor.last_control_timestamp_set = true;
            inference::executor::Status<SPEC> status = {};
            status.OK = false;
            status.TIMESTAMP_INVALID = true;
            return status;
        }
        // hierarchy: fastest/hottest loop => slowest loop
        // observation averaging: between inference control steps
        // inference control steps: maintaining policy state
        // original/training fequency control steps: advancing the policy state


        TIMESTAMP time_diff_obs = nanoseconds - executor.last_observation_timestamp; // last_control_timestamp + time_diff_previous_obs + time_diff_obs = nanoseconds
        TIMESTAMP time_diff_previous_obs = executor.last_observation_timestamp - executor.last_control_timestamp;
        TIMESTAMP time_diff_control = nanoseconds - executor.last_control_timestamp;

        inference::executor::Status<SPEC> status = {};
        status.OK = true;
        status.source = inference::executor::Status<SPEC>::OBSERVATION;
        if(executor.last_control_timestamp > executor.last_observation_timestamp){
            status.OK = false;
            status.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP = true;
            return status;
        }
        if(executor.last_control_timestamp == executor.last_observation_timestamp || time_diff_control == 0){
            // if we controlled last time, reset observation average
            copy(device, device, observation, executor.observation);
        }
        else{
            // otherwise weighted average of the intermediate observations
            T obs_weight = (T)time_diff_obs / (T)time_diff_control;
            T prev_obs_weight = (T)time_diff_previous_obs / (T)time_diff_control;
            scale(device, executor.observation, prev_obs_weight);
            for (TI obs_i = 0; obs_i < OBS_SPEC::SHAPE::LAST; obs_i++){
                T new_value = get(device, observation, 0, obs_i);
                T prev = get(device, executor.observation, 0, obs_i);
                set(device, executor.observation, prev + new_value * obs_weight, 0, obs_i);
            }
        }
        executor.last_observation_timestamp = nanoseconds;
        status.control_reasons_intermediate.time_diff = time_diff_control >= SPEC::CONTROL_INTERVAL_INTERMEDIATE_NS;
        status.control_reasons_intermediate.force_sync = SPEC::FORCE_SYNC_INTERMEDIATE;
        status.control_reasons_intermediate.reset = reset;
        if(status.control_reasons_intermediate.time_diff || status.control_reasons_intermediate.force_sync || status.control_reasons_intermediate.reset){
            // if it is time to control according to the inference frequency
            status.source = inference::executor::Status<SPEC>::CONTROL;
            if(!reset){ // if the control is due to a reset, we can/shall not rely on time_diff_control
                executor.control_dt[executor.control_dt_index++ % SPEC::TIMING_STATS_NUM_STEPS] = time_diff_control;
            }
            executor.last_control_timestamp = nanoseconds;
            if(!executor.last_control_timestamp_original_set){
                reset = true;
                executor.last_control_timestamp_original = nanoseconds;
                executor.last_control_timestamp_original_set = true;
            }
            TIMESTAMP time_diff_control_original = nanoseconds - executor.last_control_timestamp_original;


            status.control_reasons_native.time_diff = time_diff_control_original >= SPEC::CONTROL_INTERVAL_NATIVE_NS;
            auto force_sync_native = SPEC::FORCE_SYNC_NATIVE_RUNTIME ? executor.force_sync_native : SPEC::FORCE_SYNC_NATIVE;
            status.control_reasons_native.force_sync = (force_sync_native != 0) && (executor.intermediate_step % force_sync_native == 0);
            status.control_reasons_native.reset = reset;
            // Mode<mode::Evaluation<>> mode;
            Mode<nn::layers::gru::NoAutoResetMode<mode::Evaluation<>>> mode;
            if((force_sync_native != 0 && status.control_reasons_native.force_sync) || (force_sync_native == 0 && status.control_reasons_native.time_diff) || status.control_reasons_native.reset){
                evaluate_step(device, policy, observation, executor.policy_state, action, executor.policy_buffer, rng, mode);
                executor.last_control_timestamp_original = nanoseconds;
                if(!reset){
                    executor.control_original_dt[executor.control_original_dt_index++ % SPEC::TIMING_STATS_NUM_STEPS] = time_diff_control_original;
                }
                status.step_type = inference::executor::Status<SPEC>::NATIVE;
                status.timing_jitter = inference::executor::timing_jitter_status<true>(device, executor);
                status.timing_bias = inference::executor::timing_bias_status<true>(device, executor);
                status.OK = status.OK && status.timing_jitter.OK && status.timing_bias.OK;
            }
            else{
                copy(device, device, executor.policy_state, executor.policy_state_temp);
                evaluate_step(device, policy, observation, executor.policy_state_temp, action, executor.policy_buffer, rng, mode);
                status.step_type = inference::executor::Status<SPEC>::INTERMEDIATE;
                status.timing_jitter = inference::executor::timing_jitter_status<false>(device, executor);
                status.timing_bias = inference::executor::timing_bias_status<false>(device, executor);
                status.OK = status.OK && status.timing_jitter.OK && status.timing_bias.OK;
            }
            for (TI action_i = 0; action_i < SPEC::OUTPUT_DIM; action_i++){
                T raw_value = get(device, action, 0, action_i);
                T clipped = math::clamp(device.math, raw_value, (T)-1.0, (T)1.0);
                set(device, action, clipped, 0, action_i);
            }
            executor.intermediate_step++;
        }
        return status;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
