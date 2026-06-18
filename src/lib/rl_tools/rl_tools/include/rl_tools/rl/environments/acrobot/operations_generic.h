#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_ACROBOT_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_ACROBOT_OPERATIONS_GENERIC_H
#include "acrobot.h"
#include "../operations_generic.h"
// adapted from (and tested agains) https://github.com/Farama-Foundation/Gymnasium/blob/v0.28.1/gymnasium/envs/classic_control/acrobot.py
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::acrobot {
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
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, const rl::environments::Acrobot<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, const rl::environments::Acrobot<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, const rl::environments::Acrobot<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample_initial_parameters(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters, RNG& rng){ }
    template<typename DEVICE, typename SPEC>
    static void initial_parameters(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample_initial_state(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters, typename rl::environments::Acrobot<SPEC>::State& state, RNG& rng){
        state.theta_1     = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -0.1, 0.1, rng);
        state.theta_2     = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -0.1, 0.1, rng);
        state.theta_1_dot = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -0.1, 0.1, rng);
        state.theta_2_dot = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -0.1, 0.1, rng);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample_initial_state(DEVICE& device, const rl::environments::AcrobotSwingup<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters, typename rl::environments::AcrobotSwingup<SPEC>::State& state, RNG& rng){
        using T = typename SPEC::T;
        state.theta_1     = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -math::PI<T>, math::PI<T>, rng);
        state.theta_2     = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), -math::PI<T>, math::PI<T>, rng);
        state.theta_1_dot = 0;
        state.theta_2_dot = 0;
    }
    template<typename DEVICE, typename SPEC>
    static void initial_state(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters, typename rl::environments::Acrobot<SPEC>::State& state){
        state.theta_1     = 0;
        state.theta_2     = 0;
        state.theta_1_dot = 0;
        state.theta_2_dot = 0;
    }
    namespace rl::environments::acrobot{
        template <typename T, typename PARAMS>
        RL_TOOLS_FUNCTION_PLACEMENT void dsdt(T state[4], T action, T d_state[4], const PARAMS& params){

            T m1 = params.LINK_MASS_1;
            T m2 = params.LINK_MASS_2;
            T l1 = params.LINK_LENGTH_1;
            T lc1 = params.LINK_COM_POS_1;
            T lc2 = params.LINK_COM_POS_2;
            T I1 = params.LINK_MOI;
            T I2 = params.LINK_MOI;
            T g = 9.8;
            T theta1 = state[0];
            T theta2 = state[1];
            T dtheta1 = state[2];
            T dtheta2 = state[3];
            T d1 = (
                    m1 * lc1 * lc1
                    + m2 * (l1*l1 + lc2*lc2 + 2 * l1 * lc2 * cos(theta2))
                    + I1
                    + I2
            );
            T d2 = m2 * (lc2*lc2 + l1 * lc2 * cos(theta2)) + I2;
            T phi2 = m2 * lc2 * g * cos(theta1 + theta2 - math::PI<T> / 2.0);
            T phi1 = (
                    -m2 * l1 * lc2 * dtheta2 * dtheta2 * sin(theta2)
                    - 2 * m2 * l1 * lc2 * dtheta2 * dtheta1 * sin(theta2)
                    + (m1 * lc1 + m2 * l1) * g * cos(theta1 - math::PI<T> / 2)
                    + phi2
            );

            T ddtheta2 = (
                               action + d2 / d1 * phi1 - m2 * l1 * lc2 * dtheta1*dtheta1 * sin(theta2) - phi2
                       ) / (m2 * lc2*lc2 + I2 - d2*d2 / d1);
            T ddtheta1 = -(d2 * ddtheta2 + phi1) / d1;
            d_state[0] = dtheta1;
            d_state[1] = dtheta2;
            d_state[2] = ddtheta1;
            d_state[3] = ddtheta2;
        }

        template <typename T, typename PARAMS>
        RL_TOOLS_FUNCTION_PLACEMENT void rk4(T state[4], T action, T next_state[4], T dt, const PARAMS& params){

            T k1[4], k2[4], k3[4], k4[4], y1[4], y2[4], y3[4];

            T dt2 = dt / 2.0;

            rl::environments::acrobot::dsdt(state, action, k1, params);
            for (int i = 0; i < 4; ++i){
                y1[i] = state[i] + dt2 * k1[i];
            }
            rl::environments::acrobot::dsdt(y1, action, k2, params);
            for (int i = 0; i < 4; ++i){
                y2[i] = state[i] + dt2 * k2[i];
            }
            rl::environments::acrobot::dsdt(y2, action, k3, params);
            for (int i = 0; i < 4; ++i){
                y3[i] = state[i] + dt * k3[i];
            }
            rl::environments::acrobot::dsdt(y3, action, k4, params);
            for (int i = 0; i < 4; ++i){
                next_state[i] = state[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::Acrobot<SPEC>::State& next_state, RNG& rng) {
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == 1);
        using namespace rl::environments::acrobot;
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;

        T state_flat[4] = {state.theta_1, state.theta_2, state.theta_1_dot, state.theta_2_dot};
        T next_state_flat[4];
        T action_clamped = math::clamp(device.math, get(action, 0, 0), (T)-1, (T)1);
//        action_clamped = 0;
        T action_scaled = (action_clamped + 1.0) / 2.0 * (PARAMS::MAX_TORQUE - (PARAMS::MIN_TORQUE)) + (PARAMS::MIN_TORQUE);
        rl::environments::acrobot::rk4(state_flat, action_scaled, next_state_flat, PARAMS::DT, PARAMS{});

        next_state_flat[0] = angle_normalize(device.math, next_state_flat[0]);
        next_state_flat[1] = angle_normalize(device.math, next_state_flat[1]);
        next_state_flat[2] = math::clamp(    device.math, next_state_flat[2], -PARAMS::MAX_VEL_1, PARAMS::MAX_VEL_1);
        next_state_flat[3] = math::clamp(    device.math, next_state_flat[3], -PARAMS::MAX_VEL_2, PARAMS::MAX_VEL_2);

        next_state.theta_1 = next_state_flat[0];
        next_state.theta_2 = next_state_flat[1];
        next_state.theta_1_dot = next_state_flat[2];
        next_state.theta_2_dot = next_state_flat[3];

        return SPEC::PARAMETERS::DT;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, const typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::Acrobot<SPEC>::State& next_state, RNG& rng){
        return -1;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::AcrobotSwingup<SPEC>& env, const typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::Acrobot<SPEC>::State& next_state, RNG& rng){
        using T = typename SPEC::T;
//        return (-cos(state.theta_1) * SPEC::PARAMETERS::LINK_LENGTH_1 - cos(state.theta_2 + state.theta_1) * SPEC::PARAMETERS::LINK_LENGTH_2) * 0.01;
        typename DEVICE::SPEC::MATH dm;
        T a_angle_cost = rl::environments::acrobot::angle_normalize(dm, next_state.theta_1 - math::PI<T>);
//        a_angle_cost *= a_angle_cost;
        a_angle_cost = math::abs(device.math, a_angle_cost);
        T b_angle_cost = rl::environments::acrobot::angle_normalize(dm, next_state.theta_2);
//        b_angle_cost *= b_angle_cost;
        b_angle_cost = math::abs(device.math, b_angle_cost);

        T a_vel_cost = next_state.theta_1_dot;
        a_vel_cost *= a_vel_cost;
        T b_vel_cost = next_state.theta_2_dot;
        b_vel_cost *= b_vel_cost;
        T torque_cost = get(action, 0, 0);
        torque_cost *= torque_cost;

//        return math::exp(dm, -a_angle_cost) + math::exp(dm, -b_angle_cost) + SPEC::PARAMETERS::VEL_PENALTY * math::exp(dm, -a_vel_cost) + SPEC::PARAMETERS::VEL_PENALTY*math::exp(dm, -b_vel_cost) + 0.3 * math::exp(dm, -torque_cost);
        return (-a_angle_cost -b_angle_cost)/100*SPEC::PARAMETERS::DT/0.02;
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, const typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State& state, const rl::environments::acrobot::Observation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 6);
        typedef typename SPEC::T T;
        set(observation, 0, 0, math::cos(device.math, state.theta_1));
        set(observation, 0, 1, math::sin(device.math, state.theta_1));
        set(observation, 0, 2, math::cos(device.math, state.theta_2));
        set(observation, 0, 3, math::sin(device.math, state.theta_2));
        set(observation, 0, 4, state.theta_1_dot);
        set(observation, 0, 5, state.theta_2_dot);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Acrobot<SPEC>& env, const typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State state, RNG& rng){
        return (-cos(state.theta_1) - cos(state.theta_2 + state.theta_1)) > 1.0;
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::AcrobotSwingup<SPEC>& env, const typename rl::environments::Acrobot<SPEC>::Parameters& parameters, const typename rl::environments::Acrobot<SPEC>::State state, RNG& rng){
        return false; //math::abs(typename DEVICE::SPEC::MATH{}, state.theta_1_dot) > SPEC::PARAMETERS::MAX_VEL_1 || math::abs(typename DEVICE::SPEC::MATH{}, state.theta_1_dot) > SPEC::PARAMETERS::MAX_VEL_2;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
