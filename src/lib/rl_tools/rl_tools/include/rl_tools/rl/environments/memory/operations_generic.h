#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MEMORY_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MEMORY_OPERATIONS_GENERIC_H
#include "environment.h"
#include "../operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(DEVICE& device, const rl::environments::Memory<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(DEVICE& device, const rl::environments::Memory<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(DEVICE& device, const rl::environments::Memory<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, RNG& rng){ }
    template<typename DEVICE, typename SPEC>
    static void initial_parameters(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC>
    static void initial_state(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, typename rl::environments::Memory<SPEC>::State& state){
        using TI = typename DEVICE::index_t;
        for(TI step_i = 0; step_i < SPEC::PARAMETERS::HORIZON; step_i++){
            state.history[step_i] = 0;
        }
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, typename rl::environments::Memory<SPEC>::State& state, RNG& rng){
        using T = typename SPEC::T;
        initial_state(device, env, parameters, state);
        state.history[SPEC::PARAMETERS::HORIZON - 1] = random::uniform_real_distribution(device.random, (T)0, (T)1.0, rng) < SPEC::PARAMETERS::INPUT_PROBABILITY ? 1 : 0;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, const typename rl::environments::Memory<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::Memory<SPEC>::State& next_state, RNG& rng) {
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == 1);
        using namespace rl::environments::memory;
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        if constexpr(SPEC::PARAMETERS::HORIZON > 1){
            for(TI step_i = 0; step_i < SPEC::PARAMETERS::HORIZON - 1; step_i++){
                next_state.history[step_i] = state.history[step_i + 1];
            }
        }
        next_state.history[SPEC::PARAMETERS::HORIZON - 1] = random::uniform_real_distribution(device.random, (T)0, (T)1.0, rng) < SPEC::PARAMETERS::INPUT_PROBABILITY ? 1 : 0;
        return 0;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, const typename rl::environments::Memory<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::Memory<SPEC>::State& next_state, RNG& rng){
        using namespace rl::environments::memory;
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        T u_normalised = math::clamp(device.math, get(action, 0, 0), (T)-1, (T)1);
        T count = 0;
        for(TI step_i = 0; step_i < SPEC::PARAMETERS::HORIZON; step_i++){
            if(state.history[step_i] == 1){
                count++;
            }
        }
        T target = count / 10; //(SPEC::PARAMETERS::HORIZON * SPEC::PARAMETERS::INPUT_PROBABILITY) / 5.0;
        T diff = u_normalised - target;
        return - math::abs(device.math, diff) * 10;
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Memory<SPEC>& env, const typename rl::environments::Memory<SPEC>::Parameters& parameters, const typename rl::environments::Memory<SPEC>::State& state, const typename rl::environments::memory::Observation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 1);
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        set(observation, 0, 0, state.history[SPEC::PARAMETERS::HORIZON - 1]);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, const typename rl::environments::Memory<SPEC>::State state, RNG& rng){
        return false;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
