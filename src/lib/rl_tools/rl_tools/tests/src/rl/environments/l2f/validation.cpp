#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/l2f/operations_multitask_generic_forward.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/operations_multitask_generic.h>

#include <nlohmann/json.hpp>
#include <fstream>

#include <gtest/gtest.h>

#include "../../../utils/utils.h"

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = double;
using TI = typename DEVICE::index_t;


constexpr T EPSILON = 1e-6;

namespace static_parameter_builder{
    // to prevent spamming the global namespace
    using namespace rl_tools::rl::environments::l2f;
    static constexpr bool CLOSED_FORM = true;
    struct ENVIRONMENT_STATIC_PARAMETERS{
        static constexpr TI N_SUBSTEPS = 1;
        static constexpr TI ACTION_HISTORY_LENGTH = 16;
        static constexpr TI EPISODE_STEP_LIMIT = 500;
        using STATE_BASE = StateBase<StateSpecification<T, TI>>;
        using STATE_TYPE = DefaultActionHistoryState<T, TI, ACTION_HISTORY_LENGTH, 0, CLOSED_FORM>;// StateRotorsHistory<StateRotorsHistorySpecification<T, TI, ACTION_HISTORY_LENGTH, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>;
        using OBSERVATION_TYPE = DefaultActionHistoryObservation<T, TI, ACTION_HISTORY_LENGTH>;
        using OBSERVATION_TYPE_PRIVILEGED = DefaultObservation<T, TI, 0, observation::RandomForce<observation::RandomForceSpecification<T, TI, observation::RotorSpeeds<observation::RotorSpeedsSpecification<T, TI>> >>>;
        static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
        using PARAMETER_FACTORY = parameters::DEFAULT_PARAMETERS_FACTORY<T, TI>;
        static constexpr auto PARAMETER_VALUES = PARAMETER_FACTORY::nominal_parameters;
        using PARAMETERS = typename PARAMETER_FACTORY::PARAMETERS_TYPE;
        static constexpr T STATE_LIMIT_POSITION = 100000;
        static constexpr T STATE_LIMIT_VELOCITY = 100000;
        static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
    };
}

using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI, static_parameter_builder::ENVIRONMENT_STATIC_PARAMETERS>;
using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;


template <typename DEVICE>
ENVIRONMENT::State parse_state(DEVICE& device, ENVIRONMENT& env, ENVIRONMENT::State& base_state, ENVIRONMENT::Parameters& parameters, std::vector<T> step_state){
    ENVIRONMENT::State state = base_state;
    rlt::initial_state(device, env, parameters, state);
    state.position[0] = step_state[0];
    state.position[1] = step_state[1];
    state.position[2] = step_state[2];
    state.orientation[0] = step_state[3];
    state.orientation[1] = step_state[4];
    state.orientation[2] = step_state[5];
    state.orientation[3] = step_state[6];
    state.linear_velocity[0] = step_state[7];
    state.linear_velocity[1] = step_state[8];
    state.linear_velocity[2] = step_state[9];
    state.angular_velocity[0] = step_state[10];
    state.angular_velocity[1] = step_state[11];
    state.angular_velocity[2] = step_state[12];
    state.rpm[0] = step_state[13];
    state.rpm[1] = step_state[14];
    state.rpm[2] = step_state[15];
    state.rpm[3] = step_state[16];
    return state;
}

TEST(RL_TOOLS_RL_ENVIRONMENTS_L2F, VALIDATION) {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    std::string DATA_FILE_NAME = "quad_dynamics.json";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    std::cout << "DATA_FILE_PATH: " << DATA_FILE_PATH << std::endl;
    std::ifstream ifs(DATA_FILE_PATH);
    nlohmann::json j = nlohmann::json::parse(ifs);
    // print
//    std::cout << j.dump(4) << std::endl;

    ENVIRONMENT env;
    ENVIRONMENT::State state, next_state;
    ENVIRONMENT::Parameters parameters;
    rlt::malloc(device, env);
    rlt::initial_parameters(device, env, parameters);
    rlt::initial_state(device, env, parameters, state);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM, false>> action;
    parameters.dynamics.mass = j["dynamics"]["mass"];
    T radius = j["dynamics"]["radius"];
    parameters.dynamics.J[0][0] = j["dynamics"]["J"]["data"][0];
    parameters.dynamics.J[1][1] = j["dynamics"]["J"]["data"][4];
    parameters.dynamics.J[2][2] = j["dynamics"]["J"]["data"][8];
    parameters.dynamics.J_inv[0][0] = 1.0 / parameters.dynamics.J[0][0];
    parameters.dynamics.J_inv[1][1] = 1.0 / parameters.dynamics.J[1][1];
    parameters.dynamics.J_inv[2][2] = 1.0 / parameters.dynamics.J[2][2];
    for (TI rotor_i = 0; rotor_i < ENVIRONMENT::Parameters::N; rotor_i++){
        parameters.dynamics.rotor_thrust_coefficients[rotor_i][0] = j["dynamics"]["thrust_curve"]["data"][rotor_i][0];
        parameters.dynamics.rotor_thrust_coefficients[rotor_i][1] = j["dynamics"]["thrust_curve"]["data"][rotor_i][1];
        parameters.dynamics.rotor_thrust_coefficients[rotor_i][2] = j["dynamics"]["thrust_curve"]["data"][rotor_i][2];
        parameters.dynamics.rotor_torque_constants[rotor_i] = j["dynamics"]["torque_constant"];
        parameters.dynamics.rotor_time_constants_rising[rotor_i] = j["dynamics"]["motor_time_constant"];
        parameters.dynamics.rotor_time_constants_falling[rotor_i] = j["dynamics"]["motor_time_constant"];
    }
    parameters.dynamics.action_limit.min = 0;
    parameters.dynamics.action_limit.max = 1;

    for(TI trajectory_i = 0; trajectory_i < j["trajectories"].size(); trajectory_i++){
        auto trajectory = j["trajectories"][trajectory_i];
        std::vector<std::vector<T>> rpm_setpoints = trajectory["rpm_setpoints"];
        std::vector<std::vector<T>> states = trajectory["states"];
        constexpr TI STEP_LIMIT = 100;
        for(TI step_i = 0; step_i < rlt::math::min(device.math, states.size(), STEP_LIMIT); step_i++){
            std::vector<T> step_state = states[step_i];
            ENVIRONMENT::State target_state = parse_state(device, env, state, parameters, step_state);
            if(step_i == 0){
                state = target_state;
            }
            rlt::set(action, 0, 0, rpm_setpoints[step_i][0]);
            rlt::set(action, 0, 1, rpm_setpoints[step_i][1]);
            rlt::set(action, 0, 2, rpm_setpoints[step_i][2]);
            rlt::set(action, 0, 3, rpm_setpoints[step_i][3]);
            rlt::step(device, env, parameters, state, action, next_state, rng);
            if(step_i < states.size()-1){
                ENVIRONMENT::State target_next_state = parse_state(device, env, state, parameters, states[step_i+1]);
                std::cout << "step i: " << step_i << std::endl;
                T rpm_diff = 0;
                for(TI rotor_i = 0; rotor_i < 4; rotor_i++) {
                    rpm_diff += rlt::math::abs(device.math, (T)(target_next_state.rpm[0] - next_state.rpm[0]));
                }
                std::cout << "RPM diff: " << rpm_diff << std::endl;
                ASSERT_NEAR(target_next_state.rpm[0], next_state.rpm[0], EPSILON);
                ASSERT_NEAR(target_next_state.rpm[1], next_state.rpm[1], EPSILON);
                ASSERT_NEAR(target_next_state.rpm[2], next_state.rpm[2], EPSILON);

                T angular_velocity_diff = 0;
                for(TI i = 0; i < 3; i++) {
                    angular_velocity_diff += rlt::math::abs(device.math, (T)(target_next_state.angular_velocity[i] - next_state.angular_velocity[i]));
                }
                std::cout << "Angular velocity diff: " << angular_velocity_diff << std::endl;

                ASSERT_NEAR(target_next_state.angular_velocity[0], next_state.angular_velocity[0], EPSILON);
                ASSERT_NEAR(target_next_state.angular_velocity[1], next_state.angular_velocity[1], EPSILON);
                ASSERT_NEAR(target_next_state.angular_velocity[2], next_state.angular_velocity[2], EPSILON);

                T linear_velocity_diff = 0;
                for(TI i = 0; i < 3; i++) {
                    linear_velocity_diff += rlt::math::abs(device.math, (T)(target_next_state.linear_velocity[i] - next_state.linear_velocity[i]));
                }
                std::cout << "Linear velocity diff: " << linear_velocity_diff << std::endl;

                std::cout << "linear velocity update: " << state.linear_velocity[0] << " => " << next_state.linear_velocity[0] << " (target: " << target_next_state.linear_velocity[0] << ")" << std::endl;

                ASSERT_NEAR(target_next_state.linear_velocity[0], next_state.linear_velocity[0], EPSILON);
                ASSERT_NEAR(target_next_state.linear_velocity[1], next_state.linear_velocity[1], EPSILON);
                ASSERT_NEAR(target_next_state.linear_velocity[2], next_state.linear_velocity[2], EPSILON);

                T orientation_diff = 0;
                for(TI i = 0; i < 4; i++) {
                    orientation_diff += rlt::math::abs(device.math, (T)(target_next_state.orientation[i] - next_state.orientation[i]));
                }
                std::cout << "Orientation diff: " << orientation_diff << std::endl;

                ASSERT_NEAR(target_next_state.orientation[0], next_state.orientation[0], EPSILON);
                ASSERT_NEAR(target_next_state.orientation[1], next_state.orientation[1], EPSILON);
                ASSERT_NEAR(target_next_state.orientation[2], next_state.orientation[2], EPSILON);
                ASSERT_NEAR(target_next_state.orientation[3], next_state.orientation[3], EPSILON);

                T position_diff = 0;
                for(TI i = 0; i < 3; i++) {
                    position_diff += rlt::math::abs(device.math, (T)(target_next_state.position[i] - next_state.position[i]));
                }
                std::cout << "Position diff: " << position_diff << std::endl;

                ASSERT_NEAR(target_next_state.position[0], next_state.position[0], EPSILON);
                ASSERT_NEAR(target_next_state.position[1], next_state.position[1], EPSILON);
                ASSERT_NEAR(target_next_state.position[2], next_state.position[2], EPSILON);


            }

            state = next_state;
        }

    }



    rlt::step(device, env, parameters, state, action, next_state, rng);
}
