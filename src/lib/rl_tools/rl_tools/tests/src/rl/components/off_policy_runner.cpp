#include <rl_tools/operations/cpu.h>


#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/rl/environments/environments.h>
#include <rl_tools/rl/components/off_policy_runner/off_policy_runner.h>

#include <rl_tools/rl/environments/operations_cpu.h>
#include <rl_tools/rl/algorithms/td3/operations_cpu.h>

#include <gtest/gtest.h>


using DTYPE =  float;
constexpr DTYPE STATE_TOLERANCE = 0.00001;

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using TYPE_POLICY = rlt::numeric_types::Policy<float>;

using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = DTYPE;
using ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<DTYPE, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<DTYPE>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<ENVIRONMENT_SPEC>;
using EXPLORATION_POLICY_SPEC = rlt::nn_models::random_uniform::Specification<TYPE_POLICY, TI, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, rlt::nn_models::random_uniform::Range::MINUS_ONE_TO_ONE>;
using EXPLORATION_POLICY = rlt::nn_models::RandomUniform<EXPLORATION_POLICY_SPEC>;
constexpr TI BATCH_SIZE = 1;

using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
using MLP_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 30, rlt::nn::activation_functions::GELU, rlt::nn::activation_functions::IDENTITY>;
using MLP = rlt::nn_models::mlp::NeuralNetwork<MLP_CONFIG, rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, INPUT_SHAPE>;

using POLICIES = rl_tools::utils::Tuple<TI, EXPLORATION_POLICY, MLP>;
typedef rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT, POLICIES, rlt::rl::components::off_policy_runner::ParametersDefault<TYPE_POLICY, DEVICE::index_t>> OffPolicyRunnerSpec;
typedef rlt::rl::components::OffPolicyRunner<OffPolicyRunnerSpec> OffPolicyRunner;

TEST(RL_TOOLS_RL_ALGORITHMS_OFF_POLICY_RUNNER_TEST, TEST_0) {
    using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    DEVICE device;
    OPTIMIZER optimizer;
    MLP policy;
    rlt::malloc(device, policy);
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init_weights(device, policy, rng);
    OffPolicyRunner off_policy_runner;
    rlt::malloc(device, off_policy_runner);
    ENVIRONMENT envs[OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS];
    ENVIRONMENT::Parameters env_parameters[OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS];
    rlt::init(device, off_policy_runner);
    decltype(policy)::Buffer<OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS> policy_buffers;
    rlt::malloc(device, policy_buffers);
    for(int step_i = 0; step_i < 10000; step_i++){
        rlt::step<1>(device, off_policy_runner, policy, policy_buffers, rng);
    }
    rlt::free(device, off_policy_runner);
}

TEST(RL_TOOLS_RL_ALGORITHMS_OFF_POLICY_RUNNER_TEST, SEQUENTIAL_BATCH) {
    DEVICE device;
    EXPLORATION_POLICY policy;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    OffPolicyRunner off_policy_runner;
    rlt::malloc(device, off_policy_runner);
    ENVIRONMENT envs[OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS];
    ENVIRONMENT::Parameters env_parameters[OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS];
    ENVIRONMENT::State state, next_state;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM>> observation;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::malloc(device, observation);
    rlt::malloc(device, action);
    rlt::set_all(device, observation, 0);
    rlt::set_all(device, action, 0);
    rlt::init(device, off_policy_runner);
    decltype(policy)::Buffer<OffPolicyRunnerSpec::PARAMETERS::N_ENVIRONMENTS> policy_buffers;
    for(int step_i = 0; step_i < 10000; step_i++){
//        rlt::step(device, off_policy_runner, policy, policy_buffers, rng);
        T reward = step_i;
        bool terminated = false;
        bool truncated = step_i % 7 == 0;
        rlt::set_all(device, observation, step_i);
        rlt::set_all(device, action, step_i);
        auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, 0);
        rlt::add(device, replay_buffer, state, observation, observation, action, reward, next_state, observation, observation, terminated, truncated);
    }
    constexpr TI SEQUENCE_LENGTH = 10;
    rlt::rl::components::off_policy_runner::SequentialBatch<rlt::rl::components::off_policy_runner::SequentialBatchSpecification<OffPolicyRunnerSpec, SEQUENCE_LENGTH, BATCH_SIZE>> batch;
    rlt::malloc(device, batch);

    rlt::gather_batch(device, off_policy_runner, batch, rng);

    for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
        TI previous_number;
        for(TI seq_step_i = 0; seq_step_i < SEQUENCE_LENGTH; seq_step_i++){
            T reward = rlt::get(device, batch.rewards, seq_step_i, batch_i, 0);
            bool reset = rlt::get(device, batch.reset, seq_step_i, batch_i, 0);
            if(seq_step_i > 0 && !reset){
                ASSERT_EQ(previous_number+1, (TI)reward);
            }
            T action = rlt::get(device, batch.actions_current, seq_step_i, batch_i, 0);
            T observation = rlt::get(device, batch.observations_current, seq_step_i, batch_i, 0);
            T observation_priv = rlt::get(device, batch.observations_privileged_current, seq_step_i, batch_i, 0);
            std::cout << "roa: " << reward << " | " << observation << " | " << observation_priv << " | " << action << " reset: " << reset << std::endl;
            previous_number = reward;
        }
    }

    rlt::free(device, off_policy_runner);
}

