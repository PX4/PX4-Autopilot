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

using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI>;
using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;

TEST(RL_TOOLS_RL_ENVIRONMENTS_L2F, IMPORT_EXPORT){
    DEVICE device;
    RNG rng;
    rlt::init(device);
    rlt::malloc(device, rng);
    TI seed = 0;
    rlt::init(device, rng, seed);

    ENVIRONMENT env;
    ENVIRONMENT::Parameters params, params_reconstruct;
    params = {};
    ENVIRONMENT::State state, state_reconstruct;

    rlt::malloc(device, env);
    env.parameters.trajectory.langevin.alpha = 1337;
    env.parameters.dynamics.rotor_positions[0][0] = 1337;
    rlt::init(device, env);
    env.parameters.observation_delay.linear_velocity = 1337;
    env.parameters.observation_delay.angular_velocity = 1338;
    unsigned char* raw = reinterpret_cast<unsigned char*>(&params);
    unsigned char* raw_reconstruct = reinterpret_cast<unsigned char*>(&params_reconstruct);
    std::generate(raw, raw + sizeof(params), [] { return static_cast<unsigned char>(std::rand() % 256); });
    std::generate(raw_reconstruct, raw_reconstruct + sizeof(params_reconstruct), [] { return static_cast<unsigned char>(std::rand() % 256); });
    rlt::sample_initial_parameters(device, env, params, rng);
    ASSERT_NE(params.trajectory.langevin.alpha, 1337);
    ASSERT_NE(params.dynamics.rotor_positions[0][0], 1337);
    rlt::sample_initial_state(device, env, params, state, rng);
    auto params_json = rlt::json(device, env, params);
    auto params_json_reconstruct = rlt::json(device, env, params_reconstruct);
    ASSERT_NE(params_json_reconstruct, params_json);

    nlohmann::json params_json_object = nlohmann::json::parse(params_json);
    rlt::from_json(device, env, params_json, params_reconstruct);
    static constexpr T EPSILON = 1e-5;
    T abs_diff = rlt::abs_diff(device, params.dynamics, params_reconstruct.dynamics);
    ASSERT_NEAR(abs_diff, 0, EPSILON);
    if (abs_diff == 0){
        ASSERT_EQ(std::memcmp(&params, &params_reconstruct, sizeof(params)), 0);
    }
    else{
        ASSERT_NE(std::memcmp(&params, &params_reconstruct, sizeof(params)), 0);
    }

    params_json_reconstruct = rlt::json(device, env, params_reconstruct);
    ASSERT_EQ(params_json_reconstruct, params_json);
    ASSERT_NEAR(params_reconstruct.observation_delay.linear_velocity, 1337, EPSILON);
    ASSERT_NEAR(params_reconstruct.observation_delay.angular_velocity, 1338, EPSILON);

    {
        auto params_temp = params;
        params_temp.dynamics.mass += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.dynamics.J[2][2] += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.dynamics.rotor_thrust_coefficients[3][1] += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.integration.dt += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.init.max_position += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.observation_noise.position += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.action_noise.normalized_rpm += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.reward.position += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.reward.d_action += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.reward.position_clip += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.mdp.termination.position_threshold += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.disturbances.random_force.mean += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.domain_randomization.mass_max += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
    {
        auto params_temp = params;
        params_temp.domain_randomization.rotor_time_constant_falling_max += 1337;
        T diff = rlt::abs_diff(device, params, params_temp);
        ASSERT_NEAR(diff, 1337, EPSILON);
    }
}
