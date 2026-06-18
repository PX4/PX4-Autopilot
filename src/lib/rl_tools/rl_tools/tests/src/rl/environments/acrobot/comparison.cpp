#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/acrobot/operations_generic.h>
#include <rl_tools/rl/environments/operations_cpu.h>

#include "../../../utils/utils.h"
#include <gtest/gtest.h>
#include <highfive/H5File.hpp>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#define T double
const T STATE_TOLERANCE = 1e-13;

TEST(RL_TOOLS_RL_ENVIRONMENTS_ACROBOT_TEST, COMPARISON) {
    using DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    typedef rlt::rl::environments::acrobot::Specification<T, DEVICE::index_t, rlt::rl::environments::acrobot::DefaultParameters<T>> ACROBOT_SPEC;
    typedef rlt::rl::environments::Acrobot<ACROBOT_SPEC> ENVIRONMENT;
    std::string DATA_FILE_NAME = "rl_environments_acrobot_test_data.h5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    HighFive::File file(DATA_FILE_PATH, HighFive::File::ReadOnly);
    auto episodes_group = file.getGroup("episodes");
    for(TI episode_i = 0; episode_i < episodes_group.getNumberObjects(); episode_i++){
        auto episode_group = episodes_group.getGroup(std::to_string(episode_i));
        std::vector<std::vector<T>> states;
        std::vector<std::vector<T>> actions;
        std::vector<T> rewards;
        std::vector<std::vector<T>> next_states;
        std::vector<std::vector<T>> observations;
        std::vector<std::vector<T>> next_observations;

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
        state.theta_1 = states[0][0];
        state.theta_2 = states[0][1];
        state.theta_1_dot = states[0][2];
        state.theta_2_dot = states[0][3];
        for(TI step_i = 0; step_i < states.size(); step_i++){
            std::cout << "step i: " << step_i << std::endl;
            ENVIRONMENT::State next_state;
            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ENVIRONMENT::ACTION_DIM>> action;
            rlt::malloc(device, action);
            rlt::assign(device, actions[step_i].data(), action);
            rlt::step(device, env, parameters, state, action, next_state, rng);
            T r = rlt::reward(device, env, parameters, state, action, next_state, rng);
            T abs_diff = 0;
            abs_diff += abs(states[step_i][0] - state.theta_1);
            abs_diff += abs(states[step_i][1] - state.theta_2);
            abs_diff += abs(states[step_i][2] - state.theta_1_dot);
            abs_diff += abs(states[step_i][3] - state.theta_2_dot);
            abs_diff += abs(next_states[step_i][0] - next_state.theta_1);
            abs_diff += abs(next_states[step_i][1] - next_state.theta_2);
            abs_diff += abs(next_states[step_i][2] - next_state.theta_1_dot);
            abs_diff += abs(next_states[step_i][3] - next_state.theta_2_dot);
            std::cout << "abs_diff: " << abs_diff << std::endl;
            if(abs_diff > STATE_TOLERANCE){
                std::cout << "problem" << std::endl;
            }

            ASSERT_NEAR(     states[step_i][0], state.theta_1, STATE_TOLERANCE);
            ASSERT_NEAR(     states[step_i][1], state.theta_2, STATE_TOLERANCE);
            ASSERT_NEAR(     states[step_i][2], state.theta_1_dot, STATE_TOLERANCE);
            ASSERT_NEAR(     states[step_i][3], state.theta_2_dot, STATE_TOLERANCE);
            ASSERT_NEAR(    rewards[step_i]   , r, STATE_TOLERANCE);
            ASSERT_NEAR(next_states[step_i][0], next_state.theta_1, STATE_TOLERANCE);
            ASSERT_NEAR(next_states[step_i][1], next_state.theta_2, STATE_TOLERANCE);
            ASSERT_NEAR(next_states[step_i][2], next_state.theta_1_dot, STATE_TOLERANCE);
            ASSERT_NEAR(next_states[step_i][3], next_state.theta_2_dot, STATE_TOLERANCE);

            state = next_state;
        }
    }

}
