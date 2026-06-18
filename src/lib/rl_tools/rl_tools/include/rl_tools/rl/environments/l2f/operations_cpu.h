#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_CPU_H
#include "operations_generic.h"

#ifdef RL_TOOLS_ENABLE_JSON
#include <nlohmann/json.hpp>
#endif

#include <random>
#include <string>
// Why no JSON library? introduces a dependency and increases the compile time (massively in case of e.g. nlohmann::json)
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::environments::l2f::obs_helper{
        template <typename DEVICE, typename ENV, typename OBS>
        auto dispatch(DEVICE& device, const ENV& env, const OBS& obs, bool first = true);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::LastComponent<OBS_SPEC>& obs, bool first = true){
        return "";
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::Multiplex<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::Multiplex<OBS_SPEC>;
        std::string output;
        if constexpr(OBS_SPEC::ENABLE){
            output += rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::CURRENT_COMPONENT{}, first);
            first = false;
        }
        output += rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, first);
        return output;
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::Position<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::Position<OBS_SPEC>;
        return std::string(first ? "" : ".") + "Position" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::OrientationQuaternion<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::OrientationQuaternion<OBS_SPEC>;
        return std::string(first ? "" : ".") + "OrientationQuaternion" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::OrientationRotationMatrix<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::OrientationRotationMatrix<OBS_SPEC>;
        return std::string(first ? "" : ".") + "OrientationRotationMatrix" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::LinearVelocity<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::LinearVelocity<OBS_SPEC>;
        return std::string(first ? "" : ".") + "LinearVelocity" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::AngularVelocity<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::AngularVelocity<OBS_SPEC>;
        return std::string(first ? "" : ".") + "AngularVelocity" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::IMUAccelerometer<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::IMUAccelerometer<OBS_SPEC>;
        return std::string(first ? "" : ".") + "IMUAccelerometer" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::Magnetometer<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::Magnetometer<OBS_SPEC>;
        return std::string(first ? "" : ".") + "Magnetometer" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::AngularVelocityDelayed<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::AngularVelocityDelayed<OBS_SPEC>;
        return std::string(first ? "" : ".") + "AngularVelocityDelayed(" + std::to_string(OBS_SPEC::DELAY) + ")" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::PoseIntegral<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::PoseIntegral<OBS_SPEC>;
        return std::string(first ? "" : ".") + "PoseIntegral" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::RotorSpeeds<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::RotorSpeeds<OBS_SPEC>;
        return std::string(first ? "" : ".") + "RotorSpeeds" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::ActionHistory<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ActionHistory<OBS_SPEC>;
        return std::string(first ? "" : ".") + "ActionHistory(" + std::to_string(OBSERVATION::HISTORY_LENGTH) + ")" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::RandomForce<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::RandomForce<OBS_SPEC>;
        return std::string(first ? "" : ".") + "RandomForce" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::ParametersMotorPosition<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersMotorPosition<OBS_SPEC>;
        return std::string(first ? "" : ".") + "ParametersMotorPosition" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::ParametersThrustCurves<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersThrustCurves<OBS_SPEC>;
        return std::string(first ? "" : ".") + "ParametersThrustCurves" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::ParametersMass<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersMass<OBS_SPEC>;
        return std::string(first ? "" : ".") + "ParametersMass" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::ParametersInertia<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersInertia<OBS_SPEC>;
        return std::string(first ? "" : ".") + "ParametersInertia" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::TrajectoryTrackingPosition<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersInertia<OBS_SPEC>;
        return std::string(first ? "" : ".") + "TrajectoryTrackingPosition" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    template <typename DEVICE, typename SPEC, typename OBS_SPEC>
    std::string string(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::observation::TrajectoryTrackingLinearVelocity<OBS_SPEC>& obs, bool first = true){
        using OBSERVATION = rl::environments::l2f::observation::ParametersInertia<OBS_SPEC>;
        return std::string(first ? "" : ".") + "TrajectoryTrackingLinearVelocity" + rl::environments::l2f::obs_helper::dispatch(device, env, typename OBSERVATION::NEXT_COMPONENT{}, false);
    }
    namespace rl::environments::l2f::obs_helper{
        template <typename DEVICE, typename ENV, typename OBS>
        auto dispatch(DEVICE& device, const ENV& env, const OBS& obs, bool first){
            return string(device, env, obs, first);
        }
    }
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env){
        using ENVIRONMENT = rl::environments::Multirotor<SPEC>;
        std::string json_string = "{";
        json_string += "\"name\": \"l2f\",";
        json_string += "\"observation\": \"";
        json_string += rl_tools::string(device, env, typename ENVIRONMENT::Observation{});
        json_string += "\"}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename T_T, typename T_TI, T_TI N>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Dynamics<T_T, T_TI, N>& parameters){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;

        std::string json_string = "{";
        json_string += "\"rotor_positions\": [";
        for (TI i = 0; i < N; i++){
            json_string += "[" + std::to_string(parameters.rotor_positions[i][0]) + ", " + std::to_string(parameters.rotor_positions[i][1]) + ", " + std::to_string(parameters.rotor_positions[i][2]) + "]";
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_thrust_directions\": [";
        for (TI i = 0; i < N; i++){
            json_string += "[" + std::to_string(parameters.rotor_thrust_directions[i][0]) + ", " + std::to_string(parameters.rotor_thrust_directions[i][1]) + ", " + std::to_string(parameters.rotor_thrust_directions[i][2]) + "]";
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_torque_directions\": [";
        for (TI i = 0; i < N; i++){
            json_string += "[" + std::to_string(parameters.rotor_torque_directions[i][0]) + ", " + std::to_string(parameters.rotor_torque_directions[i][1]) + ", " + std::to_string(parameters.rotor_torque_directions[i][2]) + "]";
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_thrust_coefficients\": [";
        for (TI i = 0; i < N; i++){
            json_string += "[" + std::to_string(parameters.rotor_thrust_coefficients[i][0]) + ", " + std::to_string(parameters.rotor_thrust_coefficients[i][1]) + ", " + std::to_string(parameters.rotor_thrust_coefficients[i][2]) + "]";
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_torque_constants\": [";
        for (TI i = 0; i < N; i++){
            json_string += std::to_string(parameters.rotor_torque_constants[i]);
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_time_constants_rising\": [";
        for (TI i = 0; i < N; i++){
            json_string += std::to_string(parameters.rotor_time_constants_rising[i]);
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"rotor_time_constants_falling\": [";
        for (TI i = 0; i < N; i++){
            json_string += std::to_string(parameters.rotor_time_constants_falling[i]);
            if (i < N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"mass\": " + std::to_string(parameters.mass) + ", ";
        json_string += "\"gravity\": [" + std::to_string(parameters.gravity[0]) + ", " + std::to_string(parameters.gravity[1]) + ", " + std::to_string(parameters.gravity[2]) + "], ";

        json_string += "\"J\": [";
        for (TI i = 0; i < 3; i++) {
            json_string += "[" + std::to_string(parameters.J[i][0]) + ", " + std::to_string(parameters.J[i][1]) + ", " + std::to_string(parameters.J[i][2]) + "]";
            if (i < 2) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"J_inv\": [";
        for (TI i = 0; i < 3; i++) {
            json_string += "[" + std::to_string(parameters.J_inv[i][0]) + ", " + std::to_string(parameters.J_inv[i][1]) + ", " + std::to_string(parameters.J_inv[i][2]) + "]";
            if (i < 2) {
                json_string += ", ";
            }
        }
        json_string += "], ";

        json_string += "\"hovering_throttle_relative\": " + std::to_string(parameters.hovering_throttle_relative) + ", ";
        json_string += "\"action_limit\": {";
        json_string += "\"min\": " + std::to_string(parameters.action_limit.min) + ", ";
        json_string += "\"max\": " + std::to_string(parameters.action_limit.max);
        json_string += "}"; // closing action_limit
        json_string += "}"; // closing main JSON
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Integration<PARAM_SPEC>& parameters) {
        return std::string("{\"dt\": ") + std::to_string(parameters.dt) + "}";
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Initialization<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"guidance\": " + std::to_string(parameters.guidance) + ", ";
        json_string += "\"max_position\": " + std::to_string(parameters.max_position) + ", ";
        json_string += "\"max_angle\": " + std::to_string(parameters.max_angle) + ", ";
        json_string += "\"max_linear_velocity\": " + std::to_string(parameters.max_linear_velocity) + ", ";
        json_string += "\"max_angular_velocity\": " + std::to_string(parameters.max_angular_velocity) + ", ";
        json_string += "\"relative_rpm\": " + std::string(parameters.relative_rpm ? "true" : "false") + ", ";
        json_string += "\"min_rpm\": " + std::to_string(parameters.min_rpm) + ", ";
        json_string += "\"max_rpm\": " + std::to_string(parameters.max_rpm);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename T>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::reward_functions::Squared<T>& parameters) {
        std::string json_string = "{";
        json_string += "\"non_negative\": " + std::string(parameters.non_negative ? "true" : "false") + ", ";
        json_string += "\"scale\": " + std::to_string(parameters.scale) + ", ";
        json_string += "\"constant\": " + std::to_string(parameters.constant) + ", ";
        json_string += "\"termination_penalty\": " + std::to_string(parameters.termination_penalty) + ", ";
        json_string += "\"position\": " + std::to_string(parameters.position) + ", ";
        json_string += "\"position_clip\": " + std::to_string(parameters.position_clip) + ", ";
        json_string += "\"orientation\": " + std::to_string(parameters.orientation) + ", ";
        json_string += "\"linear_velocity\": " + std::to_string(parameters.linear_velocity) + ", ";
        json_string += "\"angular_velocity\": " + std::to_string(parameters.angular_velocity) + ", ";
        json_string += "\"linear_acceleration\": " + std::to_string(parameters.linear_acceleration) + ", ";
        json_string += "\"angular_acceleration\": " + std::to_string(parameters.angular_acceleration) + ", ";
        json_string += "\"action\": " + std::to_string(parameters.action) + ", ";
        json_string += "\"d_action\": " + std::to_string(parameters.d_action) + ", ";
        json_string += "\"position_error_integral\": " + std::to_string(parameters.position_error_integral);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Termination<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"enabled\": " + std::string(parameters.enabled ? "true" : "false") + ", ";
        json_string += "\"position_threshold\": " + std::to_string(parameters.position_threshold) + ", ";
        json_string += "\"linear_velocity_threshold\": " + std::to_string(parameters.linear_velocity_threshold) + ", ";
        json_string += "\"angular_velocity_threshold\": " + std::to_string(parameters.angular_velocity_threshold) + ", ";
        json_string += "\"position_integral_threshold\": " + std::to_string(parameters.position_integral_threshold) + ", ";
        json_string += "\"orientation_integral_threshold\": " + std::to_string(parameters.orientation_integral_threshold);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::ObservationNoise<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"position\": " + std::to_string(parameters.position) + ", ";
        json_string += "\"orientation\": " + std::to_string(parameters.orientation) + ", ";
        json_string += "\"linear_velocity\": " + std::to_string(parameters.linear_velocity) + ", ";
        json_string += "\"angular_velocity\": " + std::to_string(parameters.angular_velocity) + ", ";
        json_string += "\"imu_acceleration\": " + std::to_string(parameters.imu_acceleration);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::ActionNoise<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"normalized_rpm\": " + std::to_string(parameters.normalized_rpm);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::MDP<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"init\": " + json(device, env, parameters.init) + ", ";
        json_string += "\"reward\": " + json(device, env, parameters.reward) + ", ";
        json_string += "\"observation_noise\": " + json(device, env, parameters.observation_noise) + ", ";
        json_string += "\"action_noise\": " + json(device, env, parameters.action_noise) + ", ";
        json_string += "\"termination\": " + json(device, env, parameters.termination);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::ParametersBase<PARAM_SPEC>& parameters, bool top_level=true){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string json_string = top_level ? "{" : "";
        json_string += "\"dynamics\": " + json(device, env, parameters.dynamics) + ", ";
        json_string += "\"integration\": " + json(device, env, parameters.integration) + ", ";
        json_string += "\"mdp\": " + json(device, env, parameters.mdp);
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Disturbances<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"random_force\": {";
        json_string += "\"mean\": " + std::to_string(parameters.random_force.mean) + ", ";
        json_string += "\"std\": " + std::to_string(parameters.random_force.std);
        json_string += "}, ";
        json_string += "\"random_torque\": {";
        json_string += "\"mean\": " + std::to_string(parameters.random_torque.mean) + ", ";
        json_string += "\"std\": " + std::to_string(parameters.random_torque.std);
        json_string += "}";
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::ParametersDisturbances<PARAM_SPEC>& parameters, bool top_level=true){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, static_cast<const typename PARAM_SPEC::NEXT_COMPONENT&>(parameters), false);
        json_string += ", \"disturbances\": " + json(device, env, parameters.disturbances);
        json_string += (top_level ? "}" : "");
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::DomainRandomization<PARAM_SPEC>& parameters) {
        std::string json_string = "{";
        json_string += "\"thrust_to_weight_min\": " + std::to_string(parameters.thrust_to_weight_min) + ", ";
        json_string += "\"thrust_to_weight_max\": " + std::to_string(parameters.thrust_to_weight_max) + ", ";
        json_string += "\"torque_to_inertia_min\": " + std::to_string(parameters.torque_to_inertia_min) + ", ";
        json_string += "\"torque_to_inertia_max\": " + std::to_string(parameters.torque_to_inertia_max) + ", ";
        json_string += "\"mass_min\": " + std::to_string(parameters.mass_min) + ", ";
        json_string += "\"mass_max\": " + std::to_string(parameters.mass_max) + ", ";
        json_string += "\"mass_size_deviation\": " + std::to_string(parameters.mass_size_deviation) + ", ";
        json_string += "\"rotor_time_constant_rising_min\": " + std::to_string(parameters.rotor_time_constant_rising_min) + ", ";
        json_string += "\"rotor_time_constant_rising_max\": " + std::to_string(parameters.rotor_time_constant_rising_max) + ", ";
        json_string += "\"rotor_time_constant_falling_min\": " + std::to_string(parameters.rotor_time_constant_falling_min) + ", ";
        json_string += "\"rotor_time_constant_falling_max\": " + std::to_string(parameters.rotor_time_constant_falling_max) + ", ";
        json_string += "\"rotor_torque_constant_min\": " + std::to_string(parameters.rotor_torque_constant_min) + ", ";
        json_string += "\"rotor_torque_constant_max\": " + std::to_string(parameters.rotor_torque_constant_max) + ", ";
        json_string += "\"orientation_offset_angle_max\": " + std::to_string(parameters.orientation_offset_angle_max) + ", ";
        json_string += "\"disturbance_force_max\": " + std::to_string(parameters.disturbance_force_max);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::ParametersDomainRandomization<PARAM_SPEC>& parameters, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, static_cast<const typename PARAM_SPEC::NEXT_COMPONENT&>(parameters), false);
        json_string += ", \"domain_randomization\": " + json(device, env, parameters.domain_randomization);
        json_string += (top_level ? "}" : "");
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename T, typename TI>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::Trajectory<T, TI>& parameters) {
        using PARAMETERS = rl::environments::l2f::parameters::Trajectory<T, TI>;
        std::string json_string = "{";
        json_string += "\"MIXTURE_N\": " + std::to_string(PARAMETERS::MIXTURE_N) + ", ";
        json_string += "\"mixture\": [";
        for (TI i = 0; i < PARAMETERS::MIXTURE_N; i++){
            json_string += std::to_string(parameters.mixture[i]);
            if (i < PARAMETERS::MIXTURE_N - 1) {
                json_string += ", ";
            }
        }
        json_string += "], ";
        json_string += "\"langevin\": {";
        json_string += "\"gamma\": " + std::to_string(parameters.langevin.gamma) + ", ";
        json_string += "\"omega\": " + std::to_string(parameters.langevin.omega) + ", ";
        json_string += "\"sigma\": " + std::to_string(parameters.langevin.sigma) + ", ";
        json_string += "\"alpha\": " + std::to_string(parameters.langevin.alpha);
        json_string += "}";
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::ParametersTrajectory<PARAM_SPEC>& parameters, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, static_cast<const typename PARAM_SPEC::NEXT_COMPONENT&>(parameters), false);
        json_string += ", \"trajectory\": " + json(device, env, parameters.trajectory);
        json_string += (top_level ? "}" : "");
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename T, typename TI>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::parameters::ObservationDelay<T, TI>& parameters) {
        std::string json_string = "{";
        json_string += "\"linear_velocity\": " + std::to_string(parameters.linear_velocity) + ", ";
        json_string += "\"angular_velocity\": " + std::to_string(parameters.angular_velocity);
        json_string += "}";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const rl::environments::l2f::ParametersObservationDelay<PARAM_SPEC>& parameters, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, static_cast<const typename PARAM_SPEC::NEXT_COMPONENT&>(parameters), false);
        json_string += ", \"observation_delay\": " + json(device, env, parameters.observation_delay);
        json_string += (top_level ? "}" : "");
        return json_string;
    }

    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateBase<STATE_SPEC>& state, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += "\"position\": [" + std::to_string(state.position[0]) + ", " + std::to_string(state.position[1]) + ", " + std::to_string(state.position[2]) + "], ";
        json_string += "\"orientation\": [" + std::to_string(state.orientation[0]) + ", " + std::to_string(state.orientation[1]) + ", " + std::to_string(state.orientation[2]) + ", " + std::to_string(state.orientation[3]) + "], ";
        json_string += "\"linear_velocity\": [" + std::to_string(state.linear_velocity[0]) + ", " + std::to_string(state.linear_velocity[1]) + ", " + std::to_string(state.linear_velocity[2]) + "], ";
        json_string += "\"angular_velocity\": [" + std::to_string(state.angular_velocity[0]) + ", " + std::to_string(state.angular_velocity[1]) + ", " + std::to_string(state.angular_velocity[2]) + "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateLastAction<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"last_action\": [";
        for (TI action_i = 0; action_i < SPEC::PARAMETERS::N; action_i++){
            json_string += std::to_string(state.last_action[action_i]);
            if (action_i < SPEC::PARAMETERS::N - 1) {
                json_string += ", ";
            }
        }
        json_string += "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateLinearAcceleration<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"linear_acceleration\": [";
        for (TI i = 0; i < 3; i++){
            json_string += std::to_string(state.linear_acceleration[i]);
            if (i < 2) {
                json_string += ", ";
            }
        }
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"angular_velocity_history\": [";
        for (TI step_i = 0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
            json_string += "[";
            for (TI dim_i = 0; dim_i < 3; dim_i++){
                json_string += std::to_string(state.angular_velocity_history[step_i][dim_i]);
                if (dim_i < 2) {
                    json_string += ", ";
                }
            }
            json_string += "]";
            if (step_i < STATE::HISTORY_MEM_LENGTH - 1) {
                json_string += ", ";
            }
        }
        json_string += "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"linear_velocity_history\": [";
        for (TI step_i = 0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
            json_string += "[";
            for (TI dim_i = 0; dim_i < 3; dim_i++){
                json_string += std::to_string(state.linear_velocity_history[step_i][dim_i]);
                if (dim_i < 2) {
                    json_string += ", ";
                }
            }
            json_string += "]";
            if (step_i < STATE::HISTORY_MEM_LENGTH - 1) {
                json_string += ", ";
            }
        }
        json_string += "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StatePoseErrorIntegral<STATE_SPEC>& state, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"position_integral\": [" + std::to_string(state.position_integral[0]) + ", " + std::to_string(state.position_integral[1]) + ", " + std::to_string(state.position_integral[2]) + "], ";
        json_string += "\"orientation_integral\": " + std::to_string(state.orientation_integral);
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateRotors<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateRotors<STATE_SPEC>;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"rpm\": [";
        for (TI action_i = 0; action_i < STATE::ACTION_DIM; action_i++){
            json_string += std::to_string(state.rpm[action_i]);
            if (action_i < STATE::ACTION_DIM - 1) {
                json_string += ", ";
            }
        }
        json_string += "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateRotorsHistory<STATE_SPEC>& state, bool top_level=true){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateRotorsHistory<STATE_SPEC>;
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"action_history\": [";
        for (TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
            json_string += "[";
            for (TI action_i = 0; action_i < SPEC::PARAMETERS::N; action_i++){
                json_string += std::to_string(state.action_history[step_i][action_i]);
                if (action_i < SPEC::PARAMETERS::N - 1) {
                    json_string += ", ";
                }
            }
            json_string += "]";
            if (step_i < STATE_SPEC::HISTORY_LENGTH - 1) {
                json_string += ", ";
            }
        }
        json_string += "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateRandomForce<STATE_SPEC>& state, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"force\": [" + std::to_string(state.force[0]) + ", " + std::to_string(state.force[1]) + ", " + std::to_string(state.force[2]) + "], ";
        json_string += "\"torque\": [" + std::to_string(state.torque[0]) + ", " + std::to_string(state.torque[1]) + ", " + std::to_string(state.torque[2]) + "]";
        json_string += top_level ? "}" : "";
        return json_string;
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    std::string json(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const rl::environments::l2f::StateTrajectory<STATE_SPEC>& state, bool top_level=true){
        std::string json_string = top_level ? "{" : "";
        json_string += json(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), false) + ", ";
        json_string += "\"trajectory\": {";
        json_string += "\"type\": ";
        switch (state.trajectory.type){
            case rl::environments::l2f::TrajectoryType::POSITION:
                json_string += "\"POSITION\"";
                break;
            case rl::environments::l2f::TrajectoryType::LANGEVIN:
                json_string += "\"LANGEVIN\"";
                json_string += ", ";
                json_string += "\"langevin\": {";
                json_string += "\"position\": [" +  std::to_string(state.trajectory.langevin.position[0]) + ", " + std::to_string(state.trajectory.langevin.position[1]) + ", " + std::to_string(state.trajectory.langevin.position[2]) + "], ";
                json_string += "\"velocity\": [" + std::to_string(state.trajectory.langevin.velocity[0]) + ", " + std::to_string(state.trajectory.langevin.velocity[1]) + ", " + std::to_string(state.trajectory.langevin.velocity[2]) + "], ";
                json_string += "\"position_raw\": [" + std::to_string(state.trajectory.langevin.position_raw[0]) + ", " + std::to_string(state.trajectory.langevin.position_raw[1]) + ", " + std::to_string(state.trajectory.langevin.position_raw[2]) + "], ";
                json_string += "\"velocity_raw\": [" + std::to_string(state.trajectory.langevin.velocity_raw[0]) + ", " + std::to_string(state.trajectory.langevin.velocity_raw[1]) + ", " + std::to_string(state.trajectory.langevin.velocity_raw[2]) + "]";
                json_string += "}";
                break;
            default:
                json_string += "\"NONE\"";
                break;
        }
        json_string += "}";
        json_string += top_level ? "}" : "";
        return json_string;
    }
#ifdef RL_TOOLS_ENABLE_JSON
    template <typename DEVICE, typename SPEC, typename T_T, typename T_TI, T_TI N>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Dynamics<T_T, T_TI, N>& parameters){
        using TI = typename DEVICE::index_t;
        for (TI i = 0; i < N; i++){
            parameters.rotor_positions[i][0] = json_object["rotor_positions"][i][0];
            parameters.rotor_positions[i][1] = json_object["rotor_positions"][i][1];
            parameters.rotor_positions[i][2] = json_object["rotor_positions"][i][2];
            parameters.rotor_thrust_directions[i][0] = json_object["rotor_thrust_directions"][i][0];
            parameters.rotor_thrust_directions[i][1] = json_object["rotor_thrust_directions"][i][1];
            parameters.rotor_thrust_directions[i][2] = json_object["rotor_thrust_directions"][i][2];
            parameters.rotor_torque_directions[i][0] = json_object["rotor_torque_directions"][i][0];
            parameters.rotor_torque_directions[i][1] = json_object["rotor_torque_directions"][i][1];
            parameters.rotor_torque_directions[i][2] = json_object["rotor_torque_directions"][i][2];
            parameters.rotor_thrust_coefficients[i][0] = json_object["rotor_thrust_coefficients"][i][0];
            parameters.rotor_thrust_coefficients[i][1] = json_object["rotor_thrust_coefficients"][i][1];
            parameters.rotor_thrust_coefficients[i][2] = json_object["rotor_thrust_coefficients"][i][2];
            parameters.rotor_torque_constants[i] = json_object["rotor_torque_constants"][i];
            parameters.rotor_time_constants_rising[i] = json_object["rotor_time_constants_rising"][i];
            parameters.rotor_time_constants_falling[i] = json_object["rotor_time_constants_falling"][i];
        }

        parameters.mass = json_object["mass"];
        parameters.gravity[0] = json_object["gravity"][0];
        parameters.gravity[1] = json_object["gravity"][1];
        parameters.gravity[2] = json_object["gravity"][2];

        for (TI i = 0; i < 3; i++) {
            for (TI j = 0; j < 3; j++) {
                parameters.J[i][j] = json_object["J"][i][j];
                parameters.J_inv[i][j] = json_object["J_inv"][i][j];
            }
        }

        parameters.hovering_throttle_relative = json_object["hovering_throttle_relative"];
        parameters.action_limit.min = json_object["action_limit"]["min"];
        parameters.action_limit.max = json_object["action_limit"]["max"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Integration<PARAM_SPEC>& parameters) {
        parameters.dt = json_object["dt"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Initialization<PARAM_SPEC>& parameters) {
        parameters.guidance = json_object["guidance"];
        parameters.max_position = json_object["max_position"];
        parameters.max_angle = json_object["max_angle"];
        parameters.max_linear_velocity = json_object["max_linear_velocity"];
        parameters.max_angular_velocity = json_object["max_angular_velocity"];
        parameters.relative_rpm = json_object["relative_rpm"];
        parameters.min_rpm = json_object["min_rpm"];
        parameters.max_rpm = json_object["max_rpm"];
    }
    template <typename DEVICE, typename SPEC, typename T>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::reward_functions::Squared<T>& parameters) {
        parameters.non_negative = json_object["non_negative"];
        parameters.scale = json_object["scale"];
        parameters.constant = json_object["constant"];
        parameters.termination_penalty = json_object["termination_penalty"];
        parameters.position = json_object["position"];
        parameters.position_clip = json_object["position_clip"];
        parameters.orientation = json_object["orientation"];
        parameters.linear_velocity = json_object["linear_velocity"];
        parameters.angular_velocity = json_object["angular_velocity"];
        parameters.linear_acceleration = json_object["linear_acceleration"];
        parameters.angular_acceleration = json_object["angular_acceleration"];
        parameters.action = json_object["action"];
        parameters.d_action = json_object["d_action"];
        parameters.position_error_integral = json_object["position_error_integral"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Termination<PARAM_SPEC>& parameters) {
        parameters.enabled = json_object["enabled"];
        parameters.position_threshold = json_object["position_threshold"];
        parameters.linear_velocity_threshold = json_object["linear_velocity_threshold"];
        parameters.angular_velocity_threshold = json_object["angular_velocity_threshold"];
        parameters.position_integral_threshold = json_object["position_integral_threshold"];
        parameters.orientation_integral_threshold = json_object["orientation_integral_threshold"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::ObservationNoise<PARAM_SPEC>& parameters) {
        parameters.position = json_object["position"];
        parameters.orientation = json_object["orientation"];
        parameters.linear_velocity = json_object["linear_velocity"];
        parameters.angular_velocity = json_object["angular_velocity"];
        parameters.imu_acceleration = json_object["imu_acceleration"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::ActionNoise<PARAM_SPEC>& parameters) {
        parameters.normalized_rpm = json_object["normalized_rpm"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::MDP<PARAM_SPEC>& parameters) {
        from_json(device, env, json_object["init"], parameters.init);
        from_json(device, env, json_object["reward"], parameters.reward);
        from_json(device, env, json_object["observation_noise"], parameters.observation_noise);
        from_json(device, env, json_object["action_noise"], parameters.action_noise);
        from_json(device, env, json_object["termination"], parameters.termination);
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::ParametersBase<PARAM_SPEC>& parameters){
        from_json(device, env, json_object["dynamics"], parameters.dynamics);
        from_json(device, env, json_object["integration"], parameters.integration);
        from_json(device, env, json_object["mdp"], parameters.mdp);
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Disturbances<PARAM_SPEC>& parameters) {
        parameters.random_force.mean = json_object["random_force"]["mean"];
        parameters.random_force.std = json_object["random_force"]["std"];
        parameters.random_torque.mean = json_object["random_torque"]["mean"];
        parameters.random_torque.std = json_object["random_torque"]["std"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::ParametersDisturbances<PARAM_SPEC>& parameters){
        from_json(device, env, json_object, static_cast<typename PARAM_SPEC::NEXT_COMPONENT&>(parameters));
        from_json(device, env, json_object["disturbances"], parameters.disturbances);
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::DomainRandomization<PARAM_SPEC>& parameters) {
        parameters.thrust_to_weight_min = json_object["thrust_to_weight_min"];
        parameters.thrust_to_weight_max = json_object["thrust_to_weight_max"];
        parameters.torque_to_inertia_min = json_object["torque_to_inertia_min"];
        parameters.torque_to_inertia_max = json_object["torque_to_inertia_max"];
        parameters.mass_min = json_object["mass_min"];
        parameters.mass_max = json_object["mass_max"];
        parameters.mass_size_deviation = json_object["mass_size_deviation"];
        parameters.rotor_time_constant_rising_min = json_object["rotor_time_constant_rising_min"];
        parameters.rotor_time_constant_rising_max = json_object["rotor_time_constant_rising_max"];
        parameters.rotor_time_constant_falling_min = json_object["rotor_time_constant_falling_min"];
        parameters.rotor_time_constant_falling_max = json_object["rotor_time_constant_falling_max"];
        parameters.rotor_torque_constant_min = json_object["rotor_torque_constant_min"];
        parameters.rotor_torque_constant_max = json_object["rotor_torque_constant_max"];
        parameters.orientation_offset_angle_max = json_object["orientation_offset_angle_max"];
        parameters.disturbance_force_max = json_object["disturbance_force_max"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::ParametersDomainRandomization<PARAM_SPEC>& parameters){
        from_json(device, env, json_object, static_cast<typename PARAM_SPEC::NEXT_COMPONENT&>(parameters));
        from_json(device, env, json_object["domain_randomization"], parameters.domain_randomization);
    }
    template <typename DEVICE, typename SPEC, typename T, typename TI>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::Trajectory<T, TI>& parameters) {
        using PARAMETERS = rl::environments::l2f::parameters::Trajectory<T, TI>;
        rl_tools::utils::assert_exit(device, json_object["MIXTURE_N"] == PARAMETERS::MIXTURE_N, "Mismatch in MIXTURE_N");
        for (TI i = 0; i < PARAMETERS::MIXTURE_N; i++){
            parameters.mixture[i] = json_object["mixture"][i];
        }
        parameters.langevin.gamma = json_object["langevin"]["gamma"];
        parameters.langevin.omega = json_object["langevin"]["omega"];
        parameters.langevin.sigma = json_object["langevin"]["sigma"];
        parameters.langevin.alpha = json_object["langevin"]["alpha"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::ParametersTrajectory<PARAM_SPEC>& parameters){
        from_json(device, env, json_object, static_cast<typename PARAM_SPEC::NEXT_COMPONENT&>(parameters));
        from_json(device, env, json_object["trajectory"], parameters.trajectory);
    }
    template <typename DEVICE, typename SPEC, typename T, typename TI>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::parameters::ObservationDelay<T, TI>& parameters) {
        parameters.linear_velocity = json_object["linear_velocity"];
        parameters.angular_velocity = json_object["angular_velocity"];
    }
    template <typename DEVICE, typename SPEC, typename PARAM_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, nlohmann::json json_object, rl::environments::l2f::ParametersObservationDelay<PARAM_SPEC>& parameters){
        from_json(device, env, json_object, static_cast<typename PARAM_SPEC::NEXT_COMPONENT&>(parameters));
        from_json(device, env, json_object["observation_delay"], parameters.observation_delay);
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, std::string json_string, PARAMETERS& parameters){
        nlohmann::json json_object = nlohmann::json::parse(json_string);
        from_json(device, env, json_object, parameters);
    }

    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateBase<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        for (TI i = 0; i < 3; i++){
            state.position[i] = json_object["position"][i];
            state.orientation[i] = json_object["orientation"][i];
            state.linear_velocity[i] = json_object["linear_velocity"][i];
            state.angular_velocity[i] = json_object["angular_velocity"][i];
        }
        state.orientation[3] = json_object["orientation"][3];
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateLastAction<STATE_SPEC>& state){
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        for (typename DEVICE::index_t action_i = 0; action_i < SPEC::PARAMETERS::N; action_i++){
            state.last_action[action_i] = json_object["last_action"][action_i];
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateLinearAcceleration<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        for (TI i = 0; i < 3; i++){
            state.linear_acceleration[i] = json_object["linear_acceleration"][i];
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>;
        from_json(device, env, parameters, json_object, static_cast<typename STATE::NEXT_COMPONENT&>(state));
        for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
            for(TI dim_i = 0; dim_i < 3; dim_i++){
                state.angular_velocity_history[step_i][dim_i] = json_object["angular_velocity_history"][step_i][dim_i];
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>;
        from_json(device, env, parameters, json_object, static_cast<typename STATE::NEXT_COMPONENT&>(state));
        for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
            for(TI dim_i = 0; dim_i < 3; dim_i++){
                state.linear_velocity_history[step_i][dim_i] = json_object["linear_velocity_history"][step_i][dim_i];
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StatePoseErrorIntegral<STATE_SPEC>& state){
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        state.position_integral[0] = json_object["position_integral"][0];
        state.position_integral[1] = json_object["position_integral"][1];
        state.position_integral[2] = json_object["position_integral"][2];
        state.orientation_integral = json_object["orientation_integral"];
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateRotors<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateRotors<STATE_SPEC>;
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        for (TI action_i = 0; action_i < STATE::ACTION_DIM; action_i++){
            state.rpm[action_i] = json_object["rpm"][action_i];
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateRotorsHistory<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        using STATE = rl::environments::l2f::StateRotorsHistory<STATE_SPEC>;
        from_json(device, env, parameters, json_object, static_cast<typename STATE::NEXT_COMPONENT&>(state));
        for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
            for(TI action_i = 0; action_i < SPEC::PARAMETERS::N; action_i++){
                state.action_history[step_i][action_i] = json_object["action_history"][step_i][action_i];
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateRandomForce<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        for (TI i = 0; i < 3; i++){
            state.force[i] = json_object["force"][i];
            state.torque[i] = json_object["torque"][i];
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, nlohmann::json json_object, rl::environments::l2f::StateTrajectory<STATE_SPEC>& state){
        using TI = typename DEVICE::index_t;
        from_json(device, env, parameters, json_object, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
        std::string type = json_object["trajectory"]["type"];
        if(type == "POSITION"){
            state.trajectory.type = rl::environments::l2f::TrajectoryType::POSITION;
        }
        if (type == "LANGEVIN"){
            state.trajectory.type = rl::environments::l2f::TrajectoryType::LANGEVIN;
            for (TI i = 0; i < 3; i++){
                state.trajectory.langevin.position[i] = json_object["trajectory"]["langevin"]["position"][i];
                state.trajectory.langevin.velocity[i] = json_object["trajectory"]["langevin"]["velocity"][i];
                state.trajectory.langevin.position_raw[i] = json_object["trajectory"]["langevin"]["position_raw"][i];
                state.trajectory.langevin.velocity_raw[i] = json_object["trajectory"]["langevin"]["velocity_raw"][i];
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE>
    void from_json(DEVICE& device, rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, std::string json_string, STATE& state){
        using TI = typename DEVICE::index_t;
        nlohmann::json json_object = nlohmann::json::parse(json_string);
        from_json(device, env, parameters, json_object, state);
    }
#endif
    template <typename DEVICE, typename SPEC>
    std::string get_ui(DEVICE& device, rl::environments::Multirotor<SPEC>& env){
        // just the body of `function render(ctx, state, action) {` (so that it can be easily processed by `new Function("ctx", "state", "action", body)`
#ifndef _MSC_VER
        std::string ui = R"RL_TOOLS_LITERAL(













import * as THREE from "three"
import {OrbitControls} from "three-orbitcontrols"
import {GLTFLoader} from "three-gltfloader"

export class CoordinateSystem{
    constructor(origin, length=1, diameter=0.01) {
        this.cs = new THREE.Group()
        const material_red = new THREE.MeshLambertMaterial({color: 0xAA0000})
        const material_green = new THREE.MeshLambertMaterial({color: 0x00AA00})
        const material_blue = new THREE.MeshLambertMaterial({color: 0x0000AA})
        const line = new THREE.BoxGeometry(length, diameter, diameter)
        var x = new THREE.Mesh( line, material_red);
        x.position.set(length/2, 0, 0)
        var y = new THREE.Mesh( line, material_green);
        y.position.set(0, length/2, 0)
        y.rotation.set(0, 0, Math.PI/2)
        var z = new THREE.Mesh( line, material_blue);
        z.position.set(0, 0, length/2)
        z.rotation.set(0, Math.PI/2, 0)
        this.cs.add(x)
        this.cs.add(y)
        this.cs.add(z)
        this.cs.position.set(origin[0], origin[1], origin[2])
    }
    get(){
        return this.cs
    }
}

function norm(a){
    return Math.sqrt(a.map(x => x**2).reduce((a, c) => a + c, 0))
}

function Matrix4FromRotMatTranspose(rotMat){
    const m = new THREE.Matrix4()
    m.set(
        rotMat[0][0], rotMat[1][0], rotMat[2][0], 0,
        rotMat[0][1], rotMat[1][1], rotMat[2][1], 0,
        rotMat[0][2], rotMat[1][2], rotMat[2][2], 0,
        0, 0, 0, 1)
    return m
}

function Matrix4FromRotMat(rotMat){
    const m = new THREE.Matrix4()
    m.set(
        rotMat[0][0], rotMat[0][1], rotMat[0][2], 0,
        rotMat[1][0], rotMat[1][1], rotMat[1][2], 0,
        rotMat[2][0], rotMat[2][1], rotMat[2][2], 0,
        0, 0, 0, 1)
    return m
}




class State{
    constructor(canvas, {devicePixelRatio, showAxes=false, capture=false, camera_position=[0.5, 0.5, 1], camera_distance=null, interactive=true, conta_url="/conta/"}){
        this.canvas = canvas
        this.IS_MOBILE = this.is_mobile();
        this.actualDevicePixelRatio = devicePixelRatio || window.devicePixelRatio
        this.devicePixelRatio = !this.IS_MOBILE ? this.actualDevicePixelRatio : Math.min(this.actualDevicePixelRatio || 1, 2)
        this.showAxes = showAxes
        this.cursor_grab = interactive // Instruct the embedding code to make the cursor a grab cursor
        this.render_tick = 0
        this.capture = capture
        this.camera_position = camera_position
        this.camera_distance = camera_distance
        this.interactive = interactive
        this.lastCanvasWidth = 0
        this.lastCanvasHeight = 0
        this.conta_url = conta_url
    }

    is_mobile() {
        const isIOS = /iP(hone|ad|od)/.test(navigator.platform) || /iPhone|iPad|iPod/.test(navigator.userAgent);
        const isAndroid = /Android/.test(navigator.userAgent);
        const isMobile = /Mobi|Android/i.test(navigator.userAgent) || 'ontouchstart' in window || navigator.maxTouchPoints > 0;
        const isTablet = /iPad|Android(?!.*Mobile)/.test(navigator.userAgent);
        return isIOS || isAndroid || isMobile || isTablet;
    }

    async initialize(){
        const width = this.canvas.width
        const height = this.canvas.height
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera( 40, width / height, 0.1, 1000 );
        this.scene.add(this.camera);

        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: !this.IS_MOBILE,
            alpha: !this.IS_MOBILE,
            powerPreference: this.IS_MOBILE ? 'low-power' : 'high-performance',
            preserveDrawingBuffer: this.capture && !this.IS_MOBILE
          });

        this.renderer.setPixelRatio(this.devicePixelRatio)
        this.renderer.setClearColor(0xffffff, 0);

        this.renderer.setSize(width/this.actualDevicePixelRatio, height/this.actualDevicePixelRatio);

        this.lastCanvasWidth = this.canvas.width
        this.lastCanvasHeight = this.canvas.height


        // canvasContainer.appendChild(this.renderer.domElement );

        this.controls = this.interactive ? new OrbitControls(this.camera, this.renderer.domElement) : null;

        // this.controls.enabled = false;
        // window.addEventListener('keydown', (event) => {
        //     if (event.key === 'Alt') {
        //         this.controls.enabled = true;
        //         this.canvas.style.cursor = "grab"
        //     }
        // });

        // window.addEventListener('keyup', (event) => {
        //     if (event.key === 'Alt') {
        //         this.controls.enabled = false;
        //         this.canvas.style.cursor = "default"
        //     }
        // });
        this.canvas.title = "Alt+Drag to rotate the camera. Alt+CTRL+Drag to move the camera."

        this.simulator = new THREE.Group()
        this.simulator.rotation.set(-Math.PI / 2, 0, Math.PI / 2, 'XYZ');

        // const axesHelper = new THREE.AxesHelper(5);
        // this.scene.add(axesHelper)
        this.scene.add(this.simulator)

        var light = new THREE.AmbientLight( 0xffffff,0.5 ); // soft white light
        this.scene.add(light);
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.4 )
        directionalLight.position.set(-100, 100, 0)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.3 )
        directionalLight.position.set(0, 100, 100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.2 )
        directionalLight.position.set(0, 100, -100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )

        // this.camera.position.set(...this.camera_position)
        // this.camera.quaternion.set(-0.14, 0.70, 0.14, 0.68)
        // this.controls.target.set(0.0, 0.0, 0.0)
        // this.controls.minDistance = 1
        // this.controls.minDistance = 5
        // this.controls.update()

        this.camera_set = false
        this.THREE = THREE
    }

}


function thrust_direction_to_quaternion(thrust_direction){
    const x = thrust_direction[0];
    const y = thrust_direction[1];
    const z = thrust_direction[2];

    const z_unit = [0.0, 0.0, 1.0];

    let cross_x = z_unit[1] * z - z_unit[2] * y;
    let cross_y = z_unit[2] * x - z_unit[0] * z;
    let cross_z = z_unit[0] * y - z_unit[1] * x;

    const dot = z_unit[0] * x + z_unit[1] * y + z_unit[2] * z;

    const angle = Math.acos(dot);

    const cross_magnitude = Math.sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
    if (cross_magnitude != 0) {
        cross_x /= cross_magnitude;
        cross_y /= cross_magnitude;
        cross_z /= cross_magnitude;
    }

    const half_angle = angle / 2.0;
    const sin_half_angle = Math.sin(half_angle);

    const qw = Math.cos(half_angle);
    const qx = cross_x * sin_half_angle;
    const qy = cross_y * sin_half_angle;
    const qz = cross_z * sin_half_angle;
    return [qw, qx, qy, qz];
}

export class DroneMesh{
  static glbCache = new Map();
  static gltfLoader = new GLTFLoader();

  constructor(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url){
    console.assert(parameters.ui)
    this.group = new THREE.Group()
    const url = `${conta_url}${parameters.ui.model}`
    
    if (DroneMesh.glbCache.has(url)) {
      this.loaded = DroneMesh.glbCache.get(url)
    } else {
      const loadingPromise = DroneMesh.gltfLoader.loadAsync(url).then((gltf) => {
        return gltf
      }).catch((error) => {
        DroneMesh.glbCache.delete(url)
        throw error
      })
      
      DroneMesh.glbCache.set(url, loadingPromise)
      this.loaded = loadingPromise
    }
    
    this.loaded.then((gltf) => {
      const object = gltf.scene.clone()
      const object_group = new THREE.Group()
      object_group.add(object)
//      if(parameters.ui.name == "x500"){
      object_group.rotation.set(Math.PI / 2, 0, Math.PI / 2, 'ZYX')
      const scale = 0.5
      object_group.scale.set(scale, scale, scale)
//      }
      this.group.add(object_group)
    })
    if (displayIMUCoordinateSystem) {
      const scale = 1 //model.mass
      const coordinateSystemLength = Math.cbrt(scale)
      const coordinateSystemThickness = 0.01 * coordinateSystemLength
      this.group.add((new CoordinateSystem([0, 0, 0], coordinateSystemLength, coordinateSystemThickness)).get())
    }
  }
  get(){
    return this.group
  }
  set_action(action){ }
}

export class DroneDefault{
    constructor(parameters, origin, displayIMUCoordinateSystem, displayActions){
        const url = window.location.href;
        const urlObj = new URL(url);
        const params = new URLSearchParams(urlObj.search);
        if(params.has('L2FDisplayActions') === true){
            displayActions = params.get('L2FDisplayActions') === "true";
        }

        // console.log(model)
        this.origin = origin
        this.parameters = parameters
        this.droneFrame = new THREE.Group()
        this.drone = new THREE.Group()
        if(origin){
            this.drone.position.set(...origin)
        }
        // this.drone.add((new CoordinateSystem()).get())
        // this.drone.add((new CoordinateSystem(10 * this.scale, 0.1 * this.scale)).get())
        this.scale = parameters.dynamics.mass
        const material = new THREE.MeshLambertMaterial({color: 0xAAAAAA})
        const clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0x00FF00})
        const counter_clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0xFF0000})

        const coordinateSystemLength = Math.cbrt(this.scale)
        const coordinateSystemThickness = 0.01 * coordinateSystemLength

        const centerSize = Math.cbrt(this.scale) / 15
        const centerForm = new THREE.BoxGeometry(centerSize, centerSize, centerSize*0.3)
        const center = new THREE.Mesh( centerForm, material);

        this.parameters.dynamics["imu_position"] = [0, 0, 0]
        this.parameters.dynamics["imu_orientation"] = [1, 0, 0, 0]

        this.imuGroup = new THREE.Group()
        this.imuGroup.position.set(...this.parameters.dynamics.imu_position)
        this.imuGroup.quaternion.set(this.parameters.dynamics.imu_orientation[1], this.parameters.dynamics.imu_orientation[2], this.parameters.dynamics.imu_orientation[3], this.parameters.dynamics.imu_orientation[0])
        if (displayIMUCoordinateSystem) {
            this.imuGroup.add((new CoordinateSystem([0, 0, 0], coordinateSystemLength, coordinateSystemThickness)).get())
        }
        this.drone.add(this.imuGroup)
        this.drone.add(center)

        this.rotors = []

        const averageArmLength = this.parameters.dynamics.rotor_positions.map(position => norm(position)).reduce((a, c) => a + c, 0) / this.parameters.dynamics.rotor_positions.length
        for(const [rotorIndex, rotor_position] of this.parameters.dynamics.rotor_positions.entries()){
            let rotorCageRadiusFactor = 1
            let rotorCageThicknessFactor = 1
            const rotorCageRadius =  averageArmLength/3 * Math.sqrt(rotorCageRadiusFactor)
            const rotorCageThickness = averageArmLength/20 * Math.sqrt(rotorCageThicknessFactor)
            const armGroup = new THREE.Group()
            const length = norm(rotor_position)
            const armDiameter = averageArmLength/10
            const armLength = length - rotorCageRadius
            const armForm = new THREE.CylinderGeometry( armDiameter/2, armDiameter/2, armLength, 8 );
            const rot = new THREE.Quaternion(); // Geometry extends in y -> transform y to relative pos
            rot.setFromUnitVectors(new THREE.Vector3(...[0, 1, 0]), (new THREE.Vector3(...rotor_position)).normalize());
            armGroup.quaternion.set(rot.x, rot.y, rot.z, rot.w)

            const arm = new THREE.Mesh(armForm, material)
            arm.position.set(0, armLength/2, 0)
            armGroup.add(arm)

            const rotorGroup = new THREE.Group()
            rotorGroup.position.set(...rotor_position)

            const thrust_orientation = thrust_direction_to_quaternion(this.parameters.dynamics.rotor_thrust_directions[rotorIndex])
            rotorGroup.quaternion.set(thrust_orientation[3], thrust_orientation[0], thrust_orientation[1], thrust_orientation[2])
            // rotorGroup.add((new CoordinateSystem([0, 0, 0], 0.1, 0.01)).get())
            const rotorCageForm = new THREE.TorusGeometry(rotorCageRadius, rotorCageThickness, 16, 32 );
            const cageMaterial = (this.parameters.dynamics.rotor_thrust_directions[rotorIndex][2] < 0 ? clockwise_rotor_material : counter_clockwise_rotor_material)// new THREE.MeshLambertMaterial({color: 0xAAAAAA})
            const rotorCage = new THREE.Mesh(rotorCageForm, cageMaterial)
            rotorGroup.add(rotorCage)

            const forceArrow = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,0 ), 0, 0x000000);
            if(displayActions){
                rotorGroup.add(forceArrow)
            }

            this.drone.add(rotorGroup)
            this.drone.add(armGroup)
            this.droneFrame.add(this.drone)
            this.rotors.push({
                forceArrow,
                rotorCage
            })
        }

    }
    get(){
        return this.droneFrame
    }
    // setState(state){
    //   const mat = Matrix4FromRotMat(state.orientation)
    //   this.droneFrame.quaternion.setFromRotationMatrix(mat)
    //   this.droneFrame.position.set(state.pose.position[0] + this.origin[0], state.pose.position[1] + this.origin[1], state.pose.position[2] + this.origin[2])
    //   const avg_rot_rate = state.rotor_states.reduce((a, c) => a + c["power"], 0)/state.rotor_states.length
    //   state.rotor_states.map((rotorState, i) => {
    //     const forceArrow = this.rotors[i].forceArrow
    //     const rotorCage = this.rotors[i].rotorCage
    //     const min_rpm = this.model.rotors[i].min_rpm
    //     const max_rpm = this.model.rotors[i].max_rpm


    //     const rot_rate = rotorState["power"]
    //     const force_magnitude = (rot_rate - avg_rot_rate)/max_rpm * 10///1000
    //     forceArrow.setDirection(new THREE.Vector3(0, 0, rot_rate)) //Math.sign(force_magnitude)))
    //     forceArrow.setLength(Math.cbrt(this.this.scale)/10) //Math.abs(force_magnitude))
    //   })
    // }
    set_action(action){
        for(let i = 0; i < 4; i++){
            const forceArrow = this.rotors[i].forceArrow
            const force_magnitude = action[i]
            forceArrow.setDirection(new THREE.Vector3(0, 0, force_magnitude))
            forceArrow.setLength(Math.cbrt(this.scale)/10)
        }
    }

}

async function drone_factory(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url){
  if(parameters.ui && parameters.ui.enable && parameters.ui.model){
    try{
      const model = new DroneMesh(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url)
      await model.loaded
      return model
    }
    catch(error){
      console.error("An error occurred:", error.message);
    }
  }
  return new DroneDefault(parameters, origin, displayIMUCoordinateSystem, displayActions)
}

export async function init(canvas, options){
    const state = new State(canvas, options)
    await state.initialize()
    return state
}
function clear_episode(ui_state){
    if(ui_state.drone){
        ui_state.simulator.remove(ui_state.drone.get())
        if(ui_state.showAxes){
            ui_state.simulator.remove(ui_state.origin_coordinate_system.get())
        }
    }
    if(ui_state.drones){
        ui_state.drones.map(drone => ui_state.simulator.remove(drone.get()))
        if(ui_state.showAxes){
            ui_state.origin_coordinate_systems.map(cs => ui_state.simulator.remove(cs.get()))
        }
    }
}
function set_camera(ui_state, distance){
    const scale = 1/Math.sqrt(ui_state.camera_position[0]**2 + ui_state.camera_position[1]**2 + ui_state.camera_position[2]**2) * distance
    if(!ui_state.camera_set){
        ui_state.camera.position.set(ui_state.camera_position[0] * scale, ui_state.camera_position[1] * scale, ui_state.camera_position[2] * scale)
        ui_state.camera.lookAt(0, 0, 0)
        ui_state.camera_set = true
        ui_state.controls.update()
    }
}
export async function episode_init(ui_state, parameters){
    let distance = (parameters.ui && parameters.ui.camera_distance) ? parameters.ui.camera_distance : Math.cbrt(parameters.dynamics.mass) * 2
    if(ui_state.camera_distance){
        distance = ui_state.camera_distance
    }
    set_camera(ui_state, distance)
    clear_episode(ui_state)
    ui_state.drone = await drone_factory(parameters, [0, 0, 0], ui_state.showAxes, false, ui_state.conta_url)
    ui_state.simulator.add(ui_state.drone.get())
    if(ui_state.showAxes){
        ui_state.origin_coordinate_system = new CoordinateSystem([0, 0, 0], 1 * scale, 0.01 * scale)
        ui_state.simulator.add(ui_state.origin_coordinate_system.get())
    }
}

export async function episode_init_multi(ui_state, parameters){
    const grid_distance = 0.0
    const grid_size = Math.ceil(Math.sqrt(parameters.length))
    let distance = (grid_distance > 0 ? grid_distance * grid_size * 2 : Math.cbrt(parameters[0].dynamics.mass))
    if(parameters.ui && parameters.ui.camera_distance){
        distance = parameters.ui.camera_distance
    }
    if(ui_state.camera_distance){
        distance = ui_state.camera_distance
    }
    set_camera(ui_state, distance)
    clear_episode(ui_state)
    ui_state.drones = []
    if(!ui_state.showAxes && ui_state.origin_coordinate_systems){
        ui_state.origin_coordinate_systems.forEach(cs => {
            ui_state.simulator.remove(cs.get())
        })
    }
    ui_state.origin_coordinate_systems = []
    await Promise.all(parameters.map(async (parameter, i) => {
        const x = (i % grid_size) * grid_distance
        const y = Math.floor(i / grid_size) * grid_distance
        const drone = await drone_factory(parameter, [x, y, 0], ui_state.showAxes, false, ui_state.conta_url)
        ui_state.simulator.add(drone.get())
        if(ui_state.showAxes){
            const cs = new CoordinateSystem([x, y, 0], 1, 0.01)
            ui_state.simulator.add(cs.get())
            ui_state.origin_coordinate_systems.push(cs)
        }
        ui_state.drones.push(drone)
    }))
}

function update_camera(ui_state){
    const currentWidth = ui_state.canvas.width
    const currentHeight = ui_state.canvas.height
    const hasResized = currentWidth !== ui_state.lastCanvasWidth || currentHeight !== ui_state.lastCanvasHeight

    if (hasResized) {
        const width = currentWidth / ui_state.devicePixelRatio;
        const height = currentHeight / ui_state.devicePixelRatio;

        if (ui_state.camera.aspect !== width / height) {
            ui_state.camera.aspect = width / height;
            ui_state.camera.updateProjectionMatrix();
        }

        if (ui_state.renderer) {
            ui_state.renderer.setPixelRatio(ui_state.devicePixelRatio);
            ui_state.renderer.setSize(width, height, false);
        }

        ui_state.lastCanvasWidth = currentWidth
        ui_state.lastCanvasHeight = currentHeight
    }

    if(ui_state.interactive && ui_state.controls){
        ui_state.controls.update()
    }
    if(ui_state.renderer){
        ui_state.renderer.render(ui_state.scene, ui_state.camera);
    }
    ui_state.render_tick += 1
}

function clip_position(scale, position){
    const extent = Math.cbrt(scale) * 300 // to maybe prevent threejs from exploding
    const max_position = extent
    const min_position = -extent
    return position.map((p) => {
        if(p > max_position){
            return max_position
        }
        else if(p < min_position){
            return min_position
        }
        else{
            return p
        }
    })
}

export async function render(ui_state, parameters, state, action) {
    if(ui_state.drone){
        ui_state.drone.get().position.set(...clip_position(parameters.dynamics.mass, state.position))
        ui_state.drone.get().quaternion.copy(new THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
    }
    update_camera(ui_state)
}

export async function render_multi(ui_state, parameters, states, actions){
    if(ui_state.drones && ui_state.drones.length == states.length){
        states.map((state, i) => {
            const action = actions[i]
            const current_parameters = parameters[i]
            ui_state.drones[i].get().position.set(...clip_position(current_parameters.dynamics.mass, state.position))
            ui_state.drones[i].get().quaternion.copy(new THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
            ui_state.drones[i].set_action(action)
        })
    }
    update_camera(ui_state)
}




















        )RL_TOOLS_LITERAL";
#else
        std::string ui = "";
#endif
        return ui;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif