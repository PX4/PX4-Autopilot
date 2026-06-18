#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_FLAG_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_FLAG_OPERATIONS_GENERIC_H
#include "environment.h"
#include "../operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(DEVICE& device, const rl::environments::Flag<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(DEVICE& device, const rl::environments::Flag<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(DEVICE& device, const rl::environments::Flag<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, RNG& rng){
        using T = typename SPEC::T;
        parameters.flag_positions[0][0] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
        parameters.flag_positions[0][1] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
        parameters.flag_positions[1][0] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
        parameters.flag_positions[1][1] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters){
        parameters.flag_positions[0][0] = SPEC::PARAMETERS::BOARD_SIZE * 1.0 / 4;
        parameters.flag_positions[0][1] = SPEC::PARAMETERS::BOARD_SIZE * 1.0 / 4;
        parameters.flag_positions[1][0] = SPEC::PARAMETERS::BOARD_SIZE * 3.0 / 4;
        parameters.flag_positions[1][1] = SPEC::PARAMETERS::BOARD_SIZE * 3.0 / 4;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, typename rl::environments::Flag<SPEC>::State& state){
        state.position[0] = SPEC::PARAMETERS::BOARD_SIZE / 2;
        state.position[1] = SPEC::PARAMETERS::BOARD_SIZE / 2;
        state.velocity[0] = 0;
        state.velocity[1] = 0;
        state.state_machine = rl::environments::Flag<SPEC>::State::StateMachine::INITIAL;
        state.step = 0;
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, typename rl::environments::Flag<SPEC>::State& state, RNG& rng){
        using T = typename SPEC::T;
        using PARAMETERS = typename rl::environments::Flag<SPEC>::Parameters;
        state.position[0] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
        state.position[1] = random::uniform_real_distribution(device.random, (T)0, SPEC::PARAMETERS::BOARD_SIZE, rng);
        state.velocity[0] = 0;
        state.velocity[1] = 0;
        if constexpr(PARAMETERS::SAMPLE_INITIAL_STATE_WITH_FLAG_1_VISITED){
            if(random::uniform_int_distribution(device.random, 0, 1, rng) == 0){
                state.state_machine = rl::environments::Flag<SPEC>::State::StateMachine::INITIAL;
            }
            else{
                state.state_machine = rl::environments::Flag<SPEC>::State::StateMachine::FLAG_1_VISITED;
            }
        }
        else{
            state.state_machine = rl::environments::Flag<SPEC>::State::StateMachine::INITIAL;
        }
        state.step = 0;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, const typename rl::environments::Flag<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::Flag<SPEC>::State& next_state, RNG& rng) {
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == 2);
        using namespace rl::environments::pendulum;
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;
        using STATE = typename rl::environments::Flag<SPEC>::State;
        T u_normalised[2];
        u_normalised[0] = math::clamp(device.math, get(action, 0, 0), (T)-1, (T)1);
        u_normalised[1] = math::clamp(device.math, get(action, 0, 1), (T)-1, (T)1);
        T u[2];
        u[0] = PARAMS::MAX_ACCELERATION * u_normalised[0];
        u[1] = PARAMS::MAX_ACCELERATION * u_normalised[1];
        next_state.position[0] = state.position[0] + state.velocity[0] * PARAMS::DT;
        next_state.position[1] = state.position[1] + state.velocity[1] * PARAMS::DT;
        next_state.velocity[0] = state.velocity[0] + u[0] * PARAMS::DT;
        next_state.velocity[1] = state.velocity[1] + u[1] * PARAMS::DT;
        next_state.position[0] = math::clamp(device.math, next_state.position[0], (T)0, PARAMS::BOARD_SIZE);
        next_state.position[1] = math::clamp(device.math, next_state.position[1], (T)0, PARAMS::BOARD_SIZE);
        next_state.velocity[0] = math::clamp(device.math, next_state.velocity[0], (T)-PARAMS::MAX_VELOCITY, PARAMS::MAX_VELOCITY);
        next_state.velocity[1] = math::clamp(device.math, next_state.velocity[1], (T)-PARAMS::MAX_VELOCITY, PARAMS::MAX_VELOCITY);

        T distance_to_flag_1 = math::sqrt(device.math, (next_state.position[0] - parameters.flag_positions[0][0]) * (next_state.position[0] - parameters.flag_positions[0][0]) + (next_state.position[1] - parameters.flag_positions[0][1]) * (next_state.position[1] - parameters.flag_positions[0][1]));
        T distance_to_flag_2 = math::sqrt(device.math, (next_state.position[0] - parameters.flag_positions[1][0]) * (next_state.position[0] - parameters.flag_positions[1][0]) + (next_state.position[1] - parameters.flag_positions[1][1]) * (next_state.position[1] - parameters.flag_positions[1][1]));
        switch(state.state_machine){
            case STATE::StateMachine::INITIAL:
                if(distance_to_flag_1 < PARAMS::FLAG_DISTANCE_THRESHOLD){
                    next_state.state_machine = STATE::StateMachine::FLAG_1_VISITED;
                }
                else{
                    next_state.state_machine = STATE::StateMachine::INITIAL;
                }
                break;
            case STATE::StateMachine::FLAG_1_VISITED:
                if(distance_to_flag_2 < PARAMS::FLAG_DISTANCE_THRESHOLD){
                    next_state.state_machine = STATE::StateMachine::FLAG_2_VISITED;
                }
                else{
                    next_state.state_machine = STATE::StateMachine::FLAG_1_VISITED;
                }
                break;
            default:
                std::cout << "unexpected state: " << static_cast<typename DEVICE::index_t>(state.state_machine) << std::endl;
                std::exit(1);
                break;
        }
        next_state.step = state.step + 1;
        return SPEC::PARAMETERS::DT;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, const typename rl::environments::Flag<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::Flag<SPEC>::State& next_state, RNG& rng){
        using T = typename SPEC::T;
        using ENVIRONMENT = rl::environments::Flag<SPEC>;
        using PARAMETERS = typename ENVIRONMENT::Parameters;
        using STATE = typename rl::environments::Flag<SPEC>::State;
        T reward = 0;
        bool visited_flag_1 = state.state_machine != STATE::StateMachine::FLAG_1_VISITED && next_state.state_machine == STATE::StateMachine::FLAG_1_VISITED;
        bool visited_flag_2 = state.state_machine != STATE::StateMachine::FLAG_2_VISITED && next_state.state_machine == STATE::StateMachine::FLAG_2_VISITED;
        if(visited_flag_1 || visited_flag_2){
            reward = 1.0 * PARAMETERS::REWARD_SCALE;
        }
        else{
            static_assert(ENVIRONMENT::EPISODE_STEP_LIMIT > 2);
            // reward = -1.0/(ENVIRONMENT::EPISODE_STEP_LIMIT - 2);
        }
        return reward;
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Flag<SPEC>& env, const typename rl::environments::Flag<SPEC>::Parameters& parameters, const typename rl::environments::Flag<SPEC>::State& state, const typename rl::environments::flag::Observation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 11);
        using T = typename SPEC::T;
        using STATE_MACHINE = typename rl::environments::Flag<SPEC>::State::StateMachine;
        set(observation, 0, 0, state.position[0]);
        set(observation, 0, 1, state.position[1]);
        set(observation, 0, 2, state.velocity[0]);
        set(observation, 0, 3, state.velocity[1]);
        switch(state.state_machine){
            case STATE_MACHINE::INITIAL:
                set(observation, 0, 4, 1);
                set(observation, 0, 5, -1);
                set(observation, 0, 6, -1);
                break;
            case STATE_MACHINE::FLAG_1_VISITED:
                set(observation, 0, 4, -1);
                set(observation, 0, 5, 1);
                set(observation, 0, 6, -1);
                break;
            case STATE_MACHINE::FLAG_2_VISITED:
                set(observation, 0, 4, -1);
                set(observation, 0, 5, -1);
                set(observation, 0, 6, 1);
                break;
        }
        if(state.step <= 0){
            set(observation, 0,  7, parameters.flag_positions[0][0]);
            set(observation, 0,  8, parameters.flag_positions[0][1]);
            set(observation, 0,  9, parameters.flag_positions[1][0]);
            set(observation, 0, 10, parameters.flag_positions[1][1]);
        }
        else{
            set(observation, 0,  7, -1);
            set(observation, 0,  8, -1);
            set(observation, 0,  9, -1);
            set(observation, 0, 10, -1);
        }
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Flag<SPEC>& env, const typename rl::environments::Flag<SPEC>::Parameters& parameters, const typename rl::environments::Flag<SPEC>::State& state, const typename rl::environments::flag::ObservationPrivileged<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng) {
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 11);
        using T = typename SPEC::T;
        observe(device, env, parameters, state, typename rl::environments::flag::Observation<OBS_TYPE_SPEC>{}, observation, rng);
        set(observation, 0,  7, parameters.flag_positions[0][0]);
        set(observation, 0,  8, parameters.flag_positions[0][1]);
        set(observation, 0,  9, parameters.flag_positions[1][0]);
        set(observation, 0, 10, parameters.flag_positions[1][1]);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Flag<SPEC>& env, typename rl::environments::Flag<SPEC>::Parameters& parameters, const typename rl::environments::Flag<SPEC>::State state, RNG& rng){
        return state.state_machine == rl::environments::Flag<SPEC>::State::StateMachine::FLAG_2_VISITED;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
