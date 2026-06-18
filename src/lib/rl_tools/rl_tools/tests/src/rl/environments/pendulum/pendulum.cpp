#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/environments.h>
#include <rl_tools/rl/environments/operations_cpu.h>

#include "../../../utils/utils.h"
#include <gtest/gtest.h>
#include <highfive/H5File.hpp>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#define DTYPE double
const DTYPE STATE_TOLERANCE = 0.00001;

TEST(RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_TEST, COMPARISON) {
    using DEVICE = rlt::devices::DefaultCPU;
    typedef rlt::rl::environments::pendulum::Specification<DTYPE, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<DTYPE>> PENDULUM_SPEC;
    typedef rlt::rl::environments::Pendulum<PENDULUM_SPEC> ENVIRONMENT;
    std::string DATA_FILE_NAME = "pendulum.hdf5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;

    DEVICE device;
    using TI = typename DEVICE::index_t;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    HighFive::File file(DATA_FILE_PATH, HighFive::File::ReadOnly);
    auto episodes_group = file.getGroup("episodes");
    for(typename DEVICE::index_t episode_i = 0; episode_i < episodes_group.getNumberObjects(); episode_i++){
        auto episode_group = episodes_group.getGroup(std::to_string(episode_i));
        std::vector<std::vector<DTYPE>> states;
        std::vector<std::vector<DTYPE>> actions;
        std::vector<DTYPE> rewards;
        std::vector<std::vector<DTYPE>> next_states;
        std::vector<std::vector<DTYPE>> observations;
        std::vector<std::vector<DTYPE>> next_observations;

        episode_group.getDataSet("states").read(states);
        episode_group.getDataSet("actions").read(actions);
        episode_group.getDataSet("rewards").read(rewards);
        episode_group.getDataSet("next_states").read(next_states);
        episode_group.getDataSet("observations").read(observations);
        episode_group.getDataSet("next_observations").read(next_observations);
        std::cout << "episode i: " << episode_i << std::endl;
        ENVIRONMENT env;
        ENVIRONMENT::State state;
        ENVIRONMENT::Parameters parameters;
        rlt::initial_parameters(device, env, parameters);
        state.theta = states[0][0];
        state.theta_dot = states[0][1];
        for(TI step_i = 0; step_i < states.size(); step_i++){
            std::cout << "step i: " << step_i << std::endl;
            ENVIRONMENT::State next_state;
            rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE::index_t, 1, ENVIRONMENT::ACTION_DIM>> action;
            rlt::malloc(device, action);
            rlt::assign(device, actions[step_i].data(), action);
            rlt::step(device, env, parameters, state, action, next_state, rng);
            DTYPE r = rlt::reward(device, env, parameters, state, action, next_state, rng);
            EXPECT_NEAR(     states[step_i][0], state.theta, STATE_TOLERANCE);
            EXPECT_NEAR(     states[step_i][1], state.theta_dot, STATE_TOLERANCE);
            EXPECT_NEAR(    rewards[step_i]   , r, STATE_TOLERANCE);
            EXPECT_NEAR(next_states[step_i][0], next_state.theta, STATE_TOLERANCE);
            EXPECT_NEAR(next_states[step_i][1], next_state.theta_dot, STATE_TOLERANCE);
            state = next_state;
        }
    }

}
