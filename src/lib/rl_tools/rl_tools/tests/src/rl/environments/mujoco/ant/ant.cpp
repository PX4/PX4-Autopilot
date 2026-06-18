#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/mujoco/ant/operations_cpu.h>

#include "../../../../utils/utils.h"

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <chrono>
#include <iostream>

#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

namespace TEST_DEFINITIONS{
    using DEVICE = rlt::devices::DefaultCPU;
    using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
    using T = double;
    using TI = typename DEVICE::index_t;
    using ENVIRONMENT_SPEC = rlt::rl::environments::mujoco::ant::Specification<T, TI, rlt::rl::environments::mujoco::ant::DefaultParameters<T, TI>>;
    using ENVIRONMENT = rlt::rl::environments::mujoco::Ant<ENVIRONMENT_SPEC>;
}


TEST(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT, MAIN){
    using namespace TEST_DEFINITIONS;
    DEVICE dev;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters parameters;
    rlt::malloc(dev, env);

    RNG rng;
    rlt::malloc(dev, rng);
    rlt::init(dev, rng, 10);

    typename ENVIRONMENT::State state, next_state;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::malloc(dev, action);
    rlt::set_all(dev, action, 1);
    rlt::sample_initial_parameters(dev, env, parameters, rng);
    rlt::sample_initial_state(dev, env, parameters, state, rng);
    auto start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1; i++){
        rlt::step(dev, env, parameters, state, action, next_state, rng);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time: " << diff.count() << std::endl;

}

TEST(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT, STATE_COMPLETENESS){
    using namespace TEST_DEFINITIONS;
    DEVICE dev;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    rlt::malloc(dev, env);

    RNG rng;
    rlt::malloc(dev, rng);
    rlt::init(dev, rng, 10);

    using STATE = typename ENVIRONMENT::State;
    STATE initial_state, state, next_state_1, next_state_2, next_state_temp;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> initial_action, action;
    rlt::malloc(dev, initial_action);
    rlt::malloc(dev, action);
    std::vector<std::vector<T>> states_q;
    std::vector<std::vector<T>> states_q_dot;
    std::vector<std::vector<T>> next_states_q;
    std::vector<std::vector<T>> next_states_q_dot;
    std::vector<std::vector<T>> actions;
    std::vector<T> rewards;
    std::vector<bool> terminated;
    for(TI episode_i = 0; episode_i < 5; episode_i++){
        rlt::sample_initial_parameters(dev, env, env_parameters, rng);
        rlt::sample_initial_state(dev, env, env_parameters, state, rng);
        for(TI step_i = 0; step_i < 1000; step_i++){
            rlt::randn(dev, action, rng);
            rlt::clamp(dev, action, -1, 1);
            rlt::step(dev, env, env_parameters, state, action, next_state_temp, rng);
            {
                auto q_temp = rlt::wrap<DEVICE, T, ENVIRONMENT::SPEC::STATE_DIM_Q>(dev, (T*)state.q);
                auto q_dot_temp = rlt::wrap<DEVICE, T, ENVIRONMENT::SPEC::STATE_DIM_Q_DOT>(dev, (T*)state.q_dot);
                states_q.push_back(rlt::std_vector(dev, q_temp)[0]);
                states_q_dot.push_back(rlt::std_vector(dev, q_dot_temp)[0]);
                actions.push_back(rlt::std_vector(dev, action)[0]);
                rewards.push_back(rlt::reward(dev, env, env_parameters, state, action, next_state_temp, rng));
                terminated.push_back(rlt::terminated(dev, env, env_parameters, state, rng));
            }
            {
                auto q_temp = rlt::wrap<DEVICE, T, ENVIRONMENT::SPEC::STATE_DIM_Q>(dev, (T*)next_state_temp.q);
                auto q_dot_temp = rlt::wrap<DEVICE, T, ENVIRONMENT::SPEC::STATE_DIM_Q_DOT>(dev, (T*)next_state_temp.q_dot);
                next_states_q.push_back(rlt::std_vector(dev, q_temp)[0]);
                next_states_q_dot.push_back(rlt::std_vector(dev, q_dot_temp)[0]);
            }
            if(episode_i == 0 && step_i == 0){
                rlt::copy(dev, dev, action, initial_action);
                initial_state = state;
                next_state_1 = next_state_temp;
            }
            state = next_state_temp;
        }
    }
    state = initial_state;
    rlt::step(dev, env, env_parameters, state, initial_action, next_state_2, rng);

    T acc = 0;
    for(TI state_i=0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q; state_i++){
        acc += rlt::math::abs(typename DEVICE::SPEC::MATH(), next_state_1.q[state_i] - next_state_2.q[state_i]);
    }
    for(TI state_i=0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q_DOT; state_i++){
        acc += rlt::math::abs(typename DEVICE::SPEC::MATH(), next_state_1.q_dot[state_i] - next_state_2.q_dot[state_i]);
    }
    std::cout << "next_state_1 vs. next_state_2 abs diff: " << acc << std::endl;
    ASSERT_LT(acc, 1e-12);
}

TEST(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT, CHECK_INTERFACE){
    using namespace TEST_DEFINITIONS;
    DEVICE dev;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    rlt::malloc(dev, env);
    RNG rng;
    rlt::malloc(dev, rng);
    rlt::init(dev, rng, 10);

    std::string DATA_FILE_NAME = "tests_rl_environments_mujoco_ant_data.h5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
    std::vector<std::vector<T>> observations, next_observations, states, next_states, actions;
    std::vector<T> rewards;
    std::vector<T> terminated_flags;
    std::vector<T> truncated_flags;
    data_file.getDataSet("observations").read(observations);
    data_file.getDataSet("next_observations").read(next_observations);
    data_file.getDataSet("states").read(states);
    data_file.getDataSet("next_states").read(next_states);
    data_file.getDataSet("actions").read(actions);
    data_file.getDataSet("rewards").read(rewards);
    data_file.getDataSet("terminated_flags").read(terminated_flags);
    data_file.getDataSet("truncated_flags").read(truncated_flags);

    assert(observations.size() == next_observations.size());
    assert(observations.size() == states.size());
    assert(observations.size() == next_states.size());
    assert(observations.size() == actions.size());
    assert(observations.size() == rewards.size());
    assert(observations.size() == terminated_flags.size());
    assert(observations.size() == truncated_flags.size());

    using STATE = typename ENVIRONMENT::State;
    STATE state, initial_state, next_state, termination_check_state;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::malloc(dev, action);
    bool load_state = true;
    TI state_age = 0;
    TI episode_i = 0;
    for(TI step_i = 0; step_i < observations.size(); step_i++){
        std::cout << "step_i: " << step_i << std::endl;
        if(load_state){
            rlt::sample_initial_parameters(dev, env, env_parameters, rng);
            rlt::sample_initial_state(dev, env, env_parameters, state, rng);
            for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q; state_i++){
                state.q[state_i] = states[step_i][state_i];
                env.data->qpos[state_i] = states[step_i][state_i];
            }
            for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q_DOT; state_i++){
                state.q_dot[state_i] = states[step_i][state_i + ENVIRONMENT::SPEC::STATE_DIM_Q];
                env.data->qvel[state_i] = states[step_i][state_i + ENVIRONMENT::SPEC::STATE_DIM_Q];
            }
            mj_forward(env.model, env.data);
            load_state = false;
            state_age = 0;
            initial_state = state;
        }
        for(TI action_i = 0; action_i < ENVIRONMENT::ACTION_DIM; action_i++){
            set(action, 0, action_i, actions[step_i][action_i]);
        }
        mj_forward(env.model, env.data);
        rlt::step(dev, env, env_parameters, state, action, next_state, rng);
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q; state_i++){
            T abs_diff = rlt::math::abs(typename DEVICE::SPEC::MATH(), next_state.q[state_i] - next_states[step_i][state_i]);
            if(abs_diff > 0){
                T relative_diff = rlt::math::abs(typename DEVICE::SPEC::MATH(), next_state.q[state_i] - initial_state.q[state_i]);
                T ratio = relative_diff / abs_diff;
                std::cout << "ratio: " << ratio << std::endl;
                if(abs_diff > 1e-10 && ratio < 1e10){
                    ASSERT_TRUE(false);
                }
            }
        }
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q_DOT; state_i++){
            ASSERT_NEAR(next_state.q_dot[state_i], next_states[step_i][state_i + ENVIRONMENT::SPEC::STATE_DIM_Q], 1e-9);
        }
        T reward = rlt::reward(dev, env, env_parameters, state, action, next_state, rng);
        T reward_abs_diff = rlt::math::abs(typename DEVICE::SPEC::MATH(), reward - rewards[step_i]);
        if(reward_abs_diff > 1e-2){
            ASSERT_NEAR(reward, rewards[step_i], 1e-5);
        }
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q; state_i++){
            termination_check_state.q[state_i] = next_states[step_i][state_i];
        }
        for(TI state_i = 0; state_i < ENVIRONMENT::SPEC::STATE_DIM_Q_DOT; state_i++){
            termination_check_state.q_dot[state_i] = next_states[step_i][state_i + ENVIRONMENT::SPEC::STATE_DIM_Q];
        }
        bool terminated_flag = rlt::terminated(dev, env, env_parameters, termination_check_state, rng);
        assert(terminated_flag == (terminated_flags[step_i] == 1));
        bool truncated_flag = (episode_i == 999);
        assert(truncated_flag == (truncated_flags[step_i] == 1));
        if(truncated_flag || terminated_flag){
            episode_i = 0;
        }
        else{
            episode_i++;
        }
        if(terminated_flag || truncated_flag || state_age > 30){
            load_state = true;
        }
        else{
            state = next_state;
            state_age++;
        }
    }
}
