#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_OPERATIONS_GENERIC_H
#include "pendulum.h"
#include "../operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::pendulum {
    template <typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T clip(T x, T min, T max){
        x = x < min ? min : (x > max ? max : x);
        return x;
    }
    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T f_mod_python(const DEVICE& dev, T a, T b){
        return a - b * math::floor(dev, a / b);
    }

    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT T angle_normalize(const DEVICE& dev, T x){
        return f_mod_python(dev, (x + math::PI<T>), (2 * math::PI<T>)) - math::PI<T>;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(DEVICE& device, const rl::environments::Pendulum<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(DEVICE& device, const rl::environments::Pendulum<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(DEVICE& device, const rl::environments::Pendulum<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, RNG& rng){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC>
    static void initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::State<STATE_SPEC>& state){
        state.theta = -math::PI<typename SPEC::T>;
        state.theta_dot = 0;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC>
    static void initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateLastAction<STATE_SPEC>& state){
        initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(state));
        state.last_action = 0;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateMultiTask<STATE_SPEC>& state){
        initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(state));
        state.invert_action = false;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateMeta<STATE_SPEC>& state){
        sample_initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::StateLastAction<STATE_SPEC>&>(state));
        state.invert_action = false;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::State<STATE_SPEC>& state, RNG& rng){
        state.theta     = random::uniform_real_distribution(device.random, SPEC::PARAMETERS::initial_state_min_angle, SPEC::PARAMETERS::initial_state_max_angle, rng);
        state.theta_dot = random::uniform_real_distribution(device.random, SPEC::PARAMETERS::initial_state_min_speed, SPEC::PARAMETERS::initial_state_max_speed, rng);
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateLastAction<STATE_SPEC>& state, RNG& rng){
        sample_initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(state), rng);
        state.last_action = 0;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateMultiTask<STATE_SPEC>& state, RNG& rng){
        sample_initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(state), rng);
        state.invert_action = random::uniform_int_distribution(device.random, 0, 1, rng) == 1;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, typename rl::environments::pendulum::StateMeta<STATE_SPEC>& state, RNG& rng){
        sample_initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::StateLastAction<STATE_SPEC>&>(state), rng);
        // initial_state(device, env, parameters, static_cast<typename rl::environments::pendulum::StateLastAction<STATE_SPEC>&>(state));
        state.invert_action = random::uniform_int_distribution(device.random, 0, 1, rng) == 1;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::pendulum::State<STATE_SPEC>& next_state, RNG& rng) {
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == 1);
        using namespace rl::environments::pendulum;
        typedef typename SPEC::T T;
        typedef typename SPEC::PARAMETERS PARAMS;
        T u_normalised = math::clamp(device.math, get(action, 0, 0), (T)-1, (T)1);
        T u = PARAMS::max_torque * u_normalised;
        T g = PARAMS::g;
        T m = PARAMS::m;
        T l = PARAMS::l;
        T dt = PARAMS::dt;

        u = clip(u, -PARAMS::max_torque, PARAMS::max_torque);

        T newthdot = state.theta_dot + (3 * g / (2 * l) * math::sin(device.math, state.theta) + 3.0 / (m * l * l) * u) * dt;
        newthdot = clip(newthdot, -PARAMS::max_speed, PARAMS::max_speed);
        T newth = state.theta + newthdot * dt;

        next_state.theta = newth;
        next_state.theta_dot = newthdot;
        return SPEC::PARAMETERS::dt;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateLastAction<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::pendulum::StateLastAction<STATE_SPEC>& next_state, RNG& rng){
        using T = typename SPEC::T;
        T dt = step(device, env, parameters, static_cast<const typename rl::environments::pendulum::State<STATE_SPEC>&>(state), action, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(next_state), rng);
        next_state.last_action = get(action, 0, 0);
        return dt;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateMultiTask<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::pendulum::StateMultiTask<STATE_SPEC>& next_state, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        Matrix<matrix::Specification<T, TI, 1, 1, false>> modified_action;
        set(modified_action, 0, 0, state.invert_action ? -get(action, 0, 0) : get(action, 0, 0));
        T dt = step(device, env, parameters, static_cast<const typename rl::environments::pendulum::State<STATE_SPEC>&>(state), modified_action, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(next_state), rng);
        next_state.invert_action = state.invert_action;
        return dt;
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateMeta<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::pendulum::StateMeta<STATE_SPEC>& next_state, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        Matrix<matrix::Specification<T, TI, 1, 1, false>> modified_action;
        set(modified_action, 0, 0, state.invert_action ? -get(action, 0, 0) : get(action, 0, 0));
        T dt = step(device, env, parameters, static_cast<const typename rl::environments::pendulum::State<STATE_SPEC>&>(state), modified_action, static_cast<typename rl::environments::pendulum::State<STATE_SPEC>&>(next_state), rng);
        next_state.last_action = get(action, 0, 0);
        next_state.invert_action = state.invert_action;
        return dt;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::pendulum::State<STATE_SPEC>& next_state, RNG& rng){
        using namespace rl::environments::pendulum;
        typedef typename SPEC::T T;
        T angle_norm = angle_normalize(device.math, state.theta);
        T u_normalised = get(action, 0, 0);
        T u = SPEC::PARAMETERS::max_torque * u_normalised;
        T costs = angle_norm * angle_norm + 0.1 * state.theta_dot * state.theta_dot + 0.001 * (u * u);
        return -costs;
    }

    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationFourier<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 3);
        typedef typename SPEC::T T;
        using PARAMETERS = utils::typing::remove_reference_t<decltype(parameters)>;
        set(observation, 0, 0, math::cos(device.math, state.theta));
        set(observation, 0, 1, math::sin(device.math, state.theta));
        set(observation, 0, 2, state.theta_dot);
        if constexpr(PARAMETERS::OBSERVATION_NOISE_POSITION > 0){
            T noise_cos = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_POSITION, rng);
            increment(observation, 0, 0, noise_cos);
            T noise_sin = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_POSITION, rng);
            increment(observation, 0, 1, noise_sin);
        }
        if constexpr(PARAMETERS::OBSERVATION_NOISE_VELOCITY > 0){
            T noise_velocity = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_VELOCITY, rng);
            increment(observation, 0, 2, noise_velocity);
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationRaw<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 2);
        typedef typename SPEC::T T;
        set(observation, 0, 0, rl::environments::pendulum::angle_normalize(device.math, state.theta));
        set(observation, 0, 1, state.theta_dot);
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationPosition<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 2);
        typedef typename SPEC::T T;
        using PARAMETERS = utils::typing::remove_reference_t<decltype(parameters)>;
        set(observation, 0, 0, math::cos(device.math, state.theta));
        set(observation, 0, 1, math::sin(device.math, state.theta));
        if constexpr(PARAMETERS::OBSERVATION_NOISE_POSITION > 0){
            T noise_cos = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_POSITION, rng);
            increment(observation, 0, 0, noise_cos);
            T noise_sin = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_POSITION, rng);
            increment(observation, 0, 1, noise_sin);
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationVelocity<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using ENVIRONMENT = rl::environments::Pendulum<SPEC>;
        using PARAMETERS = typename ENVIRONMENT::Parameters;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 1);
        using T = typename SPEC::T;
        set(observation, 0, 0, state.theta_dot);
        if constexpr(PARAMETERS::OBSERVATION_NOISE_VELOCITY > 0){
            T noise_velocity = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_VELOCITY, rng);
            increment(observation, 0, 0, noise_velocity);
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateLastAction<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationVelocityLastAction<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using ENVIRONMENT = rl::environments::Pendulum<SPEC>;
        using PARAMETERS = typename ENVIRONMENT::Parameters;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 2);
        using T = typename SPEC::T;
        set(observation, 0, 0, state.theta_dot);
        set(observation, 0, 1, state.last_action);
        if constexpr(PARAMETERS::OBSERVATION_NOISE_VELOCITY > 0){
            T noise_velocity = random::normal_distribution::sample(device.random, (T)0, PARAMETERS::OBSERVATION_NOISE_VELOCITY, rng);
            increment(observation, 0, 0, noise_velocity);
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateMultiTask<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationMultiTask<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using ENVIRONMENT = rl::environments::Pendulum<SPEC>;
        using PARAMETERS = typename ENVIRONMENT::Parameters;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 3 + 1);
        using T = typename SPEC::T;
        auto upstream_view = view(device, observation, matrix::ViewSpec<1, 3>{}, 0, 0);
        observe(device, env, parameters, state, typename rl::environments::pendulum::ObservationFourier<OBS_TYPE_SPEC>{}, upstream_view, rng);
        set(observation, 0, 3, state.invert_action ? -1 : 1);
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::StateMeta<STATE_SPEC>& state, const typename rl::environments::pendulum::ObservationMeta<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using ENVIRONMENT = rl::environments::Pendulum<SPEC>;
        using PARAMETERS = typename ENVIRONMENT::Parameters;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 3 + 1);
        using T = typename SPEC::T;
        auto upstream_view = view(device, observation, matrix::ViewSpec<1, 3>{}, 0, 0);
        observe(device, env, parameters, state, typename rl::environments::pendulum::ObservationFourier<OBS_TYPE_SPEC>{}, upstream_view, rng);
        set(observation, 0, 3, state.last_action);
    }
    // get_serialized_state is not generally required, it is just used in the WASM demonstration of the project page, where serialization is needed to go from the WASM runtime to the JavaScript UI
    template<typename DEVICE, typename SPEC, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T get_serialized_state(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, const typename rl::environments::pendulum::State<STATE_SPEC>& state, typename DEVICE::index_t index){
        if(index == 0) {
            return state.theta;
        }
        else{
            return state.theta_dot;
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Pendulum<SPEC>& env, typename rl::environments::Pendulum<SPEC>::Parameters& parameters, const typename rl::environments::pendulum::State<STATE_SPEC> state, RNG& rng){
        using T = typename SPEC::T;
        return false; //random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), (T)0, (T)1, rng) > 0.9;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
