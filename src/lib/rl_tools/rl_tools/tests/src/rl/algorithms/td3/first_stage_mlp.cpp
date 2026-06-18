#include <rl_tools/operations/cpu.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/environments/environments.h>
#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/rl/algorithms/td3/td3.h>

#include <rl_tools/rl/algorithms/td3/operations_cpu.h>
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn_models/persist.h>

#include "../../../utils/utils.h"
#include "../../../utils/nn_comparison_mlp.h"

#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

std::string get_data_file_path(){
    std::string DATA_FILE_NAME = "model_first_stage.hdf5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    std::cout << "Using data file: " << DATA_FILE_PATH << std::endl;
    return DATA_FILE_PATH;
}
using T = double;
using DEVICE = rlt::devices::DefaultCPU;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
typedef rlt::rl::environments::pendulum::Specification<T, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<T>> PENDULUM_SPEC;
typedef rlt::rl::environments::Pendulum<PENDULUM_SPEC> ENVIRONMENT;
ENVIRONMENT env;

#define SKIP_FULL_TRAINING

template <typename T>
struct Dataset{
    Dataset(HighFive::Group g){
        g.getDataSet("states").read(states);
        g.getDataSet("actions").read(actions);
        g.getDataSet("next_states").read(next_states);
        g.getDataSet("rewards").read(rewards);
        g.getDataSet("terminated").read(terminated);
    };
    std::vector<std::vector<T>> states;
    std::vector<std::vector<T>> actions;
    std::vector<std::vector<T>> next_states;
    std::vector<std::vector<T>> rewards;
    std::vector<std::vector<T>> terminated;
};

template <typename DEVICE, typename GROUP, typename RB>
void load_dataset(DEVICE& device, GROUP& g, RB& rb){
    rlt::load(device, rb.observations, g, "states");
    rlt::load(device, rb.actions, g, "actions");
    rlt::load(device, rb.next_observations, g, "next_states");
    auto rT = rlt::view_transpose(device, rb.rewards);
    rlt::load(device, rT, g, "rewards");
    std::vector<std::vector<typename RB::T>> terminated_matrix;
    g.group.getDataSet("terminated").read(terminated_matrix);
    assert(terminated_matrix.size() == 1);
    auto terminated = terminated_matrix[0];
    for(TI i = 0; i < terminated.size(); i++){
        rlt::set(rb.terminated, i, 0, terminated[i] == 1);
    }
    std::vector<std::vector<typename RB::T>> truncated_matrix;
    g.group.getDataSet("truncated").read(truncated_matrix);
    assert(truncated_matrix.size() == 1);
    auto truncated = truncated_matrix[0];
    for(TI i = 0; i < truncated.size(); i++){
        rlt::set(rb.truncated, i, 0, truncated[i] == 1);
    }
    rb.position = terminated.size();
//    g.getDataSet("states").read(rb.observations.data);
//    g.getDataSet("actions").read(rb.actions.data);
//    g.getDataSet("next_states").read(rb.next_observations.data);
//    g.getDataSet("rewards").read(rb.rewards.data);
//    g.getDataSet("terminated").read(terminated);
//    g.getDataSet("truncated").read(truncated);
}

template <typename SPEC>
typename SPEC::T assign(const HighFive::Group g, rlt::nn::layers::dense::LayerForward<SPEC>& layer){
    std::vector<std::vector<typename SPEC::T>> weights;
    std::vector<typename SPEC::T> biases;
    g.getDataSet("weight").read(weights);
    g.getDataSet("bias").read(biases);
    for(TI i = 0; i < SPEC::OUTPUT_DIM; i++){
        for(TI j = 0; j < SPEC::INPUT_DIM; j++){
            layer.weights[i][j] = weights[i][j];
        }
        layer.biases[i] = biases[i];
    }
}
template <typename NT>
void assign_network(NT& network, const HighFive::Group g){
    assign(g.getGroup("0"), network.layer_1);
    assign(g.getGroup("1"), network.layer_2);
    assign(g.getGroup("2"), network.output_layer);
}

using AC_DEVICE = rlt::devices::DefaultCPU;
template <typename TYPE_POLICY>
struct TD3PendulumParameters: rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, AC_DEVICE::index_t>{
    constexpr static typename AC_DEVICE::index_t CRITIC_BATCH_SIZE = 32;
    constexpr static typename AC_DEVICE::index_t ACTOR_BATCH_SIZE = 32;
};

namespace first_stage_first_stage{
    using TD3_PARAMETERS = TD3PendulumParameters<TYPE_POLICY>;

    constexpr TI ACTOR_BATCH_SIZE = 1;
    constexpr TI CRITIC_BATCH_SIZE = 1;
    using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
    using ACTOR_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
    using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
    using CRITIC_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;

    using NN_DEVICE = rlt::devices::DefaultCPU;
    using ACTOR_CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, ACTOR_BATCH_SIZE>;
    using CRITIC_CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, CRITIC_BATCH_SIZE>;
    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t, rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    using ACTOR_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, ACTOR_CAPA, ACTOR_INPUT_SHAPE>;

    using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, rlt::nn::capability::Forward<>, ACTOR_INPUT_SHAPE>;

    using CRITIC_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, CRITIC_CAPA, CRITIC_INPUT_SHAPE>;

    using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, rlt::nn::capability::Forward<>, CRITIC_INPUT_SHAPE>;


    using TD3_SPEC = rlt::rl::algorithms::td3::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, ACTOR_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3_PARAMETERS>;
    using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<TD3_SPEC>;
}

template <typename T, typename NT>
T abs_diff_network(const NT network, const HighFive::Group g){
    T acc = 0;
    std::vector<std::vector<T>> weights;
    g.getDataSet("0/weight").read(weights);
    acc += abs_diff_matrix<T, NT::SPEC::LAYER_1::OUTPUT_DIM, NT::SPEC::LAYER_1::INPUT_DIM>(network.layer_1.weights, weights);
    return acc;
}
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_FIRST_STAGE, TEST_CRITIC_FORWARD) {
    constexpr TI BATCH_SIZE = 1;
    AC_DEVICE device;
    first_stage_first_stage::NN_DEVICE nn_device;
    first_stage_first_stage::ActorCriticType actor_critic;
//    actor_critic.actor_optimizer.parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;
//    actor_critic.critic_optimizers[0].parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;
//    actor_critic.critic_optimizers[1].parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;

    rlt::malloc(device, actor_critic);

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init(device, actor_critic, rng);
    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    auto critic_group1 = rlt::get_group(device, data_file, "critic_1");
    rlt::load(device, actor_critic.critics[0], critic_group1);
    auto critic_target_group1 = rlt::get_group(device, data_file, "critic_target_1");
    rlt::load(device, actor_critic.critics_target[0], critic_target_group1);

    Dataset<T> batch(data_file.getGroup("batch"));

    std::vector<std::vector<T>> outputs;
    data_file.getDataSet("batch_output").read(outputs);

    for(TI batch_sample_i = 0; batch_sample_i < batch.states.size(); batch_sample_i++){
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, rlt::get_last(first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::INPUT_SHAPE{})>> input;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        for (TI i = 0; i < batch.states[batch_sample_i].size(); i++) {
            rlt::set(input, 0, i, batch.states[batch_sample_i][i]);
        }
        for (TI i = 0; i < batch.actions[batch_sample_i].size(); i++) {
            rlt::set(input, 0, batch.states[batch_sample_i].size() + i, batch.actions[batch_sample_i][i]);
        }

        typename rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])>::template Buffer<> critic_buffer;
        rlt::malloc(device, critic_buffer);
        rlt::evaluate(device, actor_critic.critics[0], input, output, critic_buffer, rng);
        std::cout << "output: " << rlt::get(output, 0, 0) << std::endl;
        ASSERT_LT(abs(rlt::get(output, 0, 0) - outputs[batch_sample_i][0]), 1e-15);

        typename rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])>::template Buffer<> critic_target_buffer;
        rlt::malloc(device, critic_target_buffer);
        rlt::evaluate(device, actor_critic.critics_target[0], input, output, critic_target_buffer, rng);
        std::cout << "output: " << rlt::get(output, 0, 0) << std::endl;
        ASSERT_LT(abs(rlt::get(output, 0, 0) - outputs[batch_sample_i][0]), 1e-15);
        rlt::free(device, critic_buffer);
        rlt::free(device, critic_target_buffer);
        rlt::free(device, input);
        rlt::free(device, output);
    }

}
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_FIRST_STAGE, TEST_CRITIC_BACKWARD) {
    constexpr TI BATCH_SIZE = 1;
//    using ActorCriticSpec = rlt::rl::algorithms::td3::ActorCriticSpecification<rlt::devices::Generic, DTYPE, ENVIRONMENT, TestActorNetworkDefinition<DTYPE>, TestCriticNetworkDefinition<DTYPE>, TD3_PARAMETERS>;
//    typedef rlt::rl::algorithms::td3::ActorCritic<rlt::devices::Generic, ActorCriticSpec> ActorCriticType;
    AC_DEVICE device;
    first_stage_first_stage::NN_DEVICE nn_device;
    first_stage_first_stage::ActorCriticType actor_critic;
//    actor_critic.actor_optimizer.parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;
//    actor_critic.critic_optimizers[0].parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;
//    actor_critic.critic_optimizers[1].parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;
    typename first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::Buffer<> critic_buffers;
    typename first_stage_first_stage::ActorCriticType::SPEC::ACTOR_TYPE::Buffer<> actor_buffers;
    rlt::Matrix<rlt::matrix::Specification<T, typename DEVICE::index_t, 1, 1>> d_output_critic;
    rlt::Matrix<rlt::matrix::Specification<T, typename DEVICE::index_t, 1, rlt::get_last(first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::INPUT_SHAPE{})>> d_input_critic;
    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    OPTIMIZER optimizer;
//    optimizer.parameters = rlt::nn::optimizers::adam::default_parameters_torch<DTYPE>;

    rlt::malloc(device, actor_critic);
    rlt::malloc(device, critic_buffers);
    rlt::malloc(device, actor_buffers);
    rlt::malloc(device, d_output_critic);
    rlt::malloc(device, d_input_critic);
    rlt::malloc(device, optimizer);


    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init(device, actor_critic, rng);
    rlt::init(device, optimizer);

    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    auto critic_group1 = rlt::get_group(device, data_file, "critic_1");
    rlt::load(device, actor_critic.critics[0], critic_group1);
    auto critic_target_group1 = rlt::get_group(device, data_file, "critic_target_1");
    rlt::load(device, actor_critic.critics_target[0], critic_target_group1);

    Dataset<T> batch(data_file.getGroup("batch"));
    assert(batch.states.size() == 32);

    T loss = 0;
    rlt::reset_optimizer_state(device, optimizer, actor_critic.critics[0]);
    rlt::zero_gradient(device, actor_critic.critics[0]);
    for(TI batch_sample_i = 0; batch_sample_i < batch.states.size(); batch_sample_i++){
//        DTYPE input[first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::INPUT_DIM];
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, rlt::get_last(first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::INPUT_SHAPE{})>> input;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> output;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> target;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::malloc(device, target);
        for (TI i = 0; i < batch.states[batch_sample_i].size(); i++) {
            rlt::set(input, 0, i, batch.states[batch_sample_i][i]);
        }
        for (TI i = 0; i < batch.actions[batch_sample_i].size(); i++) {
            rlt::set(input, 0, batch.states[batch_sample_i].size() + i, batch.actions[batch_sample_i][i]);
        }
        rlt::set(target, 0, 0, 1);
        typename rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])>::template Buffer<> critic_buffer;
        rlt::malloc(device, critic_buffer);
        rlt::evaluate(device, actor_critic.critics[0], input, output, critic_buffer, rng);
        loss += rlt::nn::loss_functions::mse::evaluate(device, output, target);

//        rlt::forward_backward_mse(device, actor_critic.critics[0], input, target, critic_buffers, DTYPE(1)/32);
        {
            rlt::forward(device, actor_critic.critics[0], input, critic_buffer, rng);
            rlt::nn::loss_functions::mse::gradient(device, actor_critic.critics[0].output_layer.output, target, d_output_critic, T(1)/32);
            rlt::backward_full(device, actor_critic.critics[0], input, d_output_critic, d_input_critic, critic_buffers);
        }
        std::cout << "output: " << rlt::get(actor_critic.critics[0].output_layer.output, 0, 0) << std::endl;
        rlt::free(device, critic_buffer);
        rlt::free(device, input);
        rlt::free(device, output);
        rlt::free(device, target);
    }

    rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> critic_1_after_backward;
    rlt::malloc(device, critic_1_after_backward);
    auto critic_1_backward_group = rlt::get_group(device, data_file, "critic_1_backward");
    rlt::load(device, critic_1_after_backward, critic_1_backward_group);
    rlt::reset_forward_state(device, actor_critic.critics[0]);
    rlt::reset_forward_state(device, critic_1_after_backward);
    T diff_grad_per_weight = abs_diff_grad(device, actor_critic.critics[0], critic_1_after_backward)/first_stage_first_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
    ASSERT_LT(diff_grad_per_weight, 1e-17);

    std::cout << "diff_grad_per_weight: " << diff_grad_per_weight << std::endl;
}
namespace first_stage_second_stage{
    using TD3_PARAMETERS = TD3PendulumParameters<TYPE_POLICY>;

    using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, TD3_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
    using ACTOR_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
    using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, TD3_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
    using CRITIC_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;

    using NN_DEVICE = rlt::devices::DefaultCPU;
    using ACTOR_CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using CRITIC_CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t, rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    using ACTOR_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, ACTOR_CAPA, ACTOR_INPUT_SHAPE>;

    using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, rlt::nn::capability::Forward<>, ACTOR_INPUT_SHAPE>;

    using CRITIC_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, CRITIC_CAPA, CRITIC_INPUT_SHAPE>;

    using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, rlt::nn::capability::Forward<>, CRITIC_INPUT_SHAPE>;

//    using ActorStructureSpec = rlt::nn_models::mlp::StructureSpecification<DTYPE, DEVICE::index_t, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH, TD3_PARAMETERS::ACTOR_BATCH_SIZE>;
//    using CriticStructureSpec = rlt::nn_models::mlp::StructureSpecification<DTYPE, DEVICE::index_t, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY, TD3_PARAMETERS::CRITIC_BATCH_SIZE>;
//
//    using NN_DEVICE = rlt::devices::DefaultCPU;
//    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<DTYPE, typename DEVICE::index_t, rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<DTYPE>>;
//    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
//    using ACTOR_NETWORK_SPEC = rlt::nn_models::mlp::AdamSpecification<ActorStructureSpec>;
//    using ACTOR_TYPE = rlt::nn_models::mlp::NeuralNetworkAdam<ACTOR_NETWORK_SPEC>;
//
//    using ACTOR_TARGET_NETWORK_SPEC = rlt::nn_models::mlp::ForwardSpecification<ActorStructureSpec>;
//    using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetworkForward<ACTOR_TARGET_NETWORK_SPEC>;
//
//    using CRITIC_NETWORK_SPEC = rlt::nn_models::mlp::AdamSpecification<CriticStructureSpec>;
//    using CRITIC_TYPE = rlt::nn_models::mlp::NeuralNetworkAdam<CRITIC_NETWORK_SPEC>;
//
//    using CRITIC_TARGET_NETWORK_SPEC = rlt::nn_models::mlp::ForwardSpecification<CriticStructureSpec>;
//    using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetworkForward<CRITIC_TARGET_NETWORK_SPEC>;


    using TD3_SPEC = rlt::rl::algorithms::td3::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, ACTOR_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3_PARAMETERS>;
    using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<TD3_SPEC>;
}
template <typename TYPE_POLICY, typename TI>
struct OFF_POLICY_RUNNER_PARAMETERS: rlt::rl::components::off_policy_runner::ParametersDefault<TYPE_POLICY, TI>{
    static constexpr TI REPLAY_BUFFER_CAPACITY = 32;
    static constexpr TI EPISODE_STEP_LIMIT = 100;
    static constexpr bool STOCHASTIC_POLICY = true;
    static constexpr bool COLLECT_EPISODE_STATS = true;
    static constexpr TI EPISODE_STATS_BUFFER_SIZE = 1000;
};
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_FIRST_STAGE, TEST_CRITIC_TRAINING) {
    constexpr bool verbose = true;
//    typedef rlt::rl::algorithms::td3::ActorCriticSpecification<rlt::devices::Generic, DTYPE, ENVIRONMENT, TestActorNetworkDefinition<DTYPE>, TestCriticNetworkDefinition<DTYPE>, TD3_PARAMETERS> ActorCriticSpec;
//    typedef rlt::rl::algorithms::td3::ActorCritic<rlt::devices::Generic, ActorCriticSpec> ActorCriticType;
    AC_DEVICE device;
//    first_stage_second_stage::OPTIMIZER optimizer;
    first_stage_second_stage::NN_DEVICE nn_device;
    first_stage_second_stage::ActorCriticType actor_critic;
    rlt::malloc(device, actor_critic);

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init(device, actor_critic, rng);

    rlt::get_ref(device, actor_critic.actor_optimizer.parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[0].parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[1].parameters, 0).epsilon_sqrt = 0;

    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    auto actor_group = rlt::get_group(device, data_file, "actor");
    rlt::load(device, actor_critic.actor, actor_group);
    auto actor_target_group = rlt::get_group(device, data_file, "actor_target");
    rlt::load(device, actor_critic.actor_target, actor_target_group);
    auto critic_1_group = rlt::get_group(device, data_file, "critic_1");
    rlt::load(device, actor_critic.critics[0], critic_1_group);
    auto critic_target_1_group = rlt::get_group(device, data_file, "critic_target_1");
    rlt::load(device, actor_critic.critics_target[0], critic_target_1_group);
    auto critic_2_group = rlt::get_group(device, data_file, "critic_2");
    rlt::load(device, actor_critic.critics[1], critic_2_group);
    auto critic_target_2_group = rlt::get_group(device, data_file, "critic_target_2");
    rlt::load(device, actor_critic.critics_target[1], critic_target_2_group);

    using DEVICE = rlt::devices::DefaultCPU;
    using TI = DEVICE::index_t;
    using POLICIES = rlt::utils::Tuple<TI, first_stage_first_stage::ACTOR_TYPE>;
    using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS<TYPE_POLICY, TI>>;
    using OFF_POLICY_RUNNER_TYPE = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;
    using DEVICE = rlt::devices::DefaultCPU;
    using ReplayBufferType = OFF_POLICY_RUNNER_TYPE::REPLAY_BUFFER_TYPE;
    using ReplayBufferSpec = OFF_POLICY_RUNNER_TYPE::REPLAY_BUFFER_SPEC;
//    using ReplayBufferSpec = rlt::rl::components::replay_buffer::Specification<DTYPE, AC_DEVICE::index_t, 3, 1, 32>;
//    using ReplayBufferType = rlt::rl::components::ReplayBuffer<ReplayBufferSpec>;
    OFF_POLICY_RUNNER_TYPE off_policy_runner;
    rlt::malloc(device, off_policy_runner);
    auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, 0);
    auto batch_group = rlt::get_group(device, data_file, "batch");
    load_dataset(device, batch_group, replay_buffer);
    if(rlt::is_nan(device, replay_buffer.observations) ||rlt::is_nan(device, replay_buffer.actions) ||rlt::is_nan(device, replay_buffer.next_observations) ||rlt::is_nan(device, replay_buffer.rewards)){
        assert(false);
    }
    static_assert(first_stage_second_stage::TD3_PARAMETERS::ACTOR_BATCH_SIZE == first_stage_second_stage::TD3_PARAMETERS::CRITIC_BATCH_SIZE, "ACTOR_BATCH_SIZE must be CRITIC_BATCH_SIZE");
    replay_buffer.position = first_stage_second_stage::TD3_PARAMETERS::ACTOR_BATCH_SIZE;

    constexpr DEVICE::index_t SEQUENCE_LENGTH = 1;
    using CRITIC_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, SEQUENCE_LENGTH, decltype(actor_critic)::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>;
    rlt::rl::components::off_policy_runner::SequentialBatch<CRITIC_BATCH_SPEC> critic_batch;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<first_stage_second_stage::ActorCriticType::SPEC>> critic_training_buffers;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<first_stage_second_stage::ActorCriticType::SPEC>> critic_training_buffers_target;
    rlt::malloc(device, critic_batch);
    rlt::malloc(device, critic_training_buffers);
    rlt::malloc(device, critic_training_buffers_target);

    first_stage_second_stage::CRITIC_TYPE::Buffer<> critic_buffers[2];
    rlt::malloc(device, critic_buffers[0]);
    rlt::malloc(device, critic_buffers[1]);

    first_stage_second_stage::ACTOR_TYPE::Buffer<> actor_buffers[2];
    rlt::malloc(device, actor_buffers[0]);
    rlt::malloc(device, actor_buffers[1]);

    rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> pre_critic_1;
    rlt::malloc(device, pre_critic_1);
    rlt::reset_optimizer_state(device, actor_critic.critic_optimizers[0], actor_critic.critics[0]);
    rlt::copy(device, device, actor_critic.critics[0], pre_critic_1);
    T mean_ratio = 0;
    T mean_ratio_grad = 0;
    T mean_ratio_adam = 0;
    auto critic_training_group = rlt::get_group(device, data_file, "critic_training");
    TI num_updates = critic_training_group.group.getNumberObjects();
    for(TI training_step_i = 0; training_step_i < num_updates; training_step_i++){
        auto step_group = rlt::get_group(device, critic_training_group, std::to_string(training_step_i));
        auto next_actions_matrix_view = rlt::matrix_view(device, critic_training_buffers_target.next_actions);
        auto next_state_action_value_input_matrix_view = rlt::matrix_view(device, critic_training_buffers_target.next_state_action_value_input);
        auto next_state_action_value_critic_1_matrix_view = rlt::matrix_view(device, critic_training_buffers_target.next_state_action_value_critic_1);
        auto next_state_action_value_critic_2_matrix_view = rlt::matrix_view(device, critic_training_buffers_target.next_state_action_value_critic_2);
        auto target_action_values_matrix_view = rlt::matrix_view(device, critic_training_buffers_target.target_action_value);
        auto train_critics_group = rlt::get_group(device, step_group, "train_critics");
        rlt::load(device, next_actions_matrix_view, train_critics_group, "target_next_actions_clipped");
        rlt::load(device, next_state_action_value_input_matrix_view, train_critics_group, "next_state_action_value_input");
        rlt::load(device, next_state_action_value_critic_1_matrix_view, train_critics_group, "next_state_action_values_critic_1");
        rlt::load(device, next_state_action_value_critic_2_matrix_view, train_critics_group, "next_state_action_values_critic_2");
        rlt::load(device, target_action_values_matrix_view, train_critics_group, "target_action_values");

        rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> post_critic_1;
        rlt::malloc(device, post_critic_1);
        auto critic_group = rlt::get_group(device, step_group, "critic");
        rlt::load(device, post_critic_1, critic_group);

        std::vector<std::vector<T>> target_next_action_noise_vector;
        step_group.group.getDataSet("target_next_action_noise").read(target_next_action_noise_vector);


        for(TI i = 0; i < first_stage_second_stage::ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE; i++){
            for(TI j = 0; j < first_stage_second_stage::ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM; j++){
                rlt::set(device, critic_training_buffers.target_next_action_noise, target_next_action_noise_vector[i][j], 0, i, j);
            }
        }

        rlt::gather_batch<DEVICE, OFF_POLICY_RUNNER_SPEC, CRITIC_BATCH_SPEC, decltype(rng), true>(device, off_policy_runner, critic_batch, rng);
        if(
            rlt::is_nan(device, critic_batch.observations_current) ||
            rlt::is_nan(device, critic_batch.actions_current) ||
            rlt::is_nan(device, critic_batch.observations_next) ||
            rlt::is_nan(device, critic_batch.rewards)
        ){
            assert(false);
        }
//        assert(!rlt::is_nan(device, actor_critic));

        rlt::train_critic(device, actor_critic, actor_critic.critics[0], critic_batch, actor_critic.critic_optimizers[0], actor_buffers[0], actor_buffers[0], critic_buffers[0], critic_buffers[0], critic_training_buffers, rng);

        auto target_next_action_diff = rlt::abs_diff(device, critic_training_buffers_target.next_actions, critic_training_buffers.next_actions);
        auto next_state_action_value_input_diff = rlt::abs_diff(device, critic_training_buffers_target.next_state_action_value_input, critic_training_buffers.next_state_action_value_input);
        auto next_state_action_value_critic_1_diff = rlt::abs_diff(device, critic_training_buffers_target.next_state_action_value_critic_1, critic_training_buffers.next_state_action_value_critic_1);
        auto next_state_action_value_critic_2_diff = rlt::abs_diff(device, critic_training_buffers_target.next_state_action_value_critic_2, critic_training_buffers.next_state_action_value_critic_2);
        auto target_action_value_diff = rlt::abs_diff(device, critic_training_buffers_target.target_action_value, critic_training_buffers.target_action_value);
        ASSERT_LT(target_next_action_diff, 1e-14);
        ASSERT_LT(next_state_action_value_input_diff, 1e-14);
        ASSERT_LT(next_state_action_value_critic_1_diff, 1e-14);
        ASSERT_LT(next_state_action_value_critic_2_diff, 1e-14);
        ASSERT_LT(target_action_value_diff, 1e-14);

        rlt::reset_forward_state(device, pre_critic_1);
        rlt::reset_forward_state(device, post_critic_1);
        rlt::reset_forward_state(device, actor_critic.critics[0]);
        rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> compare_critic;
        rlt::malloc(device, compare_critic);
        rlt::copy(device, device, actor_critic.critics[0], compare_critic);

        {
            rlt::utils::typing::remove_reference_t<decltype(actor_critic.critic_optimizers[0])> reset_optimizer;
            rlt::malloc(device, reset_optimizer);
            rlt::init(device, reset_optimizer);
            rlt::reset_optimizer_state(device, reset_optimizer, pre_critic_1);
            rlt::reset_optimizer_state(device, reset_optimizer, post_critic_1);
            rlt::reset_optimizer_state(device, reset_optimizer, compare_critic);
            rlt::free(device, reset_optimizer);
        }



        T pre_post_diff_per_weight = abs_diff(device, pre_critic_1, post_critic_1)/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_target_per_weight = abs_diff(device, post_critic_1, compare_critic)/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_ratio = pre_post_diff_per_weight/diff_target_per_weight;

        T pre_post_diff_grad_per_weight = abs_diff_grad(device, pre_critic_1, post_critic_1)/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_target_grad_per_weight = abs_diff_grad(device, post_critic_1, actor_critic.critics[0])/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_ratio_grad = pre_post_diff_grad_per_weight/diff_target_grad_per_weight;

        T pre_post_diff_adam_per_weight = abs_diff_adam(device, pre_critic_1, post_critic_1)/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_target_adam_per_weight = abs_diff_adam(device, post_critic_1, compare_critic)/first_stage_second_stage::ActorCriticType::SPEC::CRITIC_TYPE::NUM_WEIGHTS;
        T diff_ratio_adam = pre_post_diff_adam_per_weight/diff_target_adam_per_weight;

        rlt::free(device, compare_critic);

        if(verbose){
            std:: cout << "    actor update" << std::endl;
//                std::cout << "        pre_post_diff_per_weight: " << pre_post_diff_per_weight << std::endl;
//                std::cout << "        diff_target_per_weight: " << diff_target_per_weight << std::endl;
            std::cout << "        update ratio     : " << diff_ratio << std::endl;

//                std::cout << "        pre_post_diff_grad_per_weight: " << pre_post_diff_grad_per_weight << std::endl;
//                std::cout << "        diff_target_grad_per_weight: " << diff_target_grad_per_weight << std::endl;
            std::cout << "        update ratio grad: " << diff_ratio_grad << std::endl;

//                std::cout << "        pre_post_diff_adam_per_weight: " << pre_post_diff_adam_per_weight << std::endl;
//                std::cout << "        diff_target_adam_per_weight: " << diff_target_adam_per_weight << std::endl;
            std::cout << "        update ratio adam: " << diff_ratio_adam << std::endl;
        }

        mean_ratio += diff_ratio;
        mean_ratio_grad += diff_ratio_grad;
        mean_ratio_adam += diff_ratio_adam;

//        ASSERT_LT(diff_target_per_weight, 1e-7);
//        actor_critic.critics[0] = post_critic;

    }
    mean_ratio /= num_updates;
    mean_ratio_grad /= num_updates;
    mean_ratio_adam /= num_updates;
    std::cout << "mean_ratio: " << mean_ratio << std::endl;
    std::cout << "mean_ratio grad: " << mean_ratio_grad << std::endl;
    std::cout << "mean_ratio adam: " << mean_ratio_adam << std::endl;
    ASSERT_GT(mean_ratio, 1e14);
    ASSERT_GT(mean_ratio_grad, 1e14);
    ASSERT_GT(mean_ratio_adam, 1e14);
    rlt::free(device, replay_buffer);
}


TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_FIRST_STAGE, TEST_ACTOR_TRAINING) {
    constexpr bool verbose = true;
//    typedef rlt::rl::algorithms::td3::ActorCriticSpecification<rlt::devices::Generic, DTYPE, ENVIRONMENT, TestActorNetworkDefinition<DTYPE>, TestCriticNetworkDefinition<DTYPE>, TD3_PARAMETERS> ActorCriticSpec;
//    typedef rlt::rl::algorithms::td3::ActorCritic<rlt::devices::Generic, ActorCriticSpec> ActorCriticType;
    AC_DEVICE device;
//    first_stage_second_stage::OPTIMIZER optimizer;
    first_stage_second_stage::NN_DEVICE nn_device;
    first_stage_second_stage::ActorCriticType actor_critic;
    rlt::malloc(device, actor_critic);

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init(device, actor_critic, rng);

    rlt::get_ref(device, actor_critic.actor_optimizer.parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[0].parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[1].parameters, 0).epsilon_sqrt = 0;

    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    auto actor_group = rlt::get_group(device, data_file, "actor");
    rlt::load(device, actor_critic.actor, actor_group);
    auto actor_target_group = rlt::get_group(device, data_file, "actor_target");
    rlt::load(device, actor_critic.actor_target, actor_target_group);
    auto critic_1_group = rlt::get_group(device, data_file, "critic_1");
    rlt::load(device, actor_critic.critics[0], critic_1_group);
    auto critic_target_1_group = rlt::get_group(device, data_file, "critic_target_1");
    rlt::load(device, actor_critic.critics_target[0], critic_target_1_group);
    auto critic_2_group = rlt::get_group(device, data_file, "critic_2");
    rlt::load(device, actor_critic.critics[1], critic_2_group);
    auto critic_target_2_group = rlt::get_group(device, data_file, "critic_target_2");
    rlt::load(device, actor_critic.critics_target[1], critic_target_2_group);

    using DEVICE = rlt::devices::DefaultCPU;
//    using ReplayBufferSpec = rlt::rl::components::replay_buffer::Specification<DTYPE, AC_DEVICE::index_t, 3, 1, 32>;
//    using ReplayBufferType = rlt::rl::components::ReplayBuffer<ReplayBufferSpec>;
    using POLICIES = rlt::utils::Tuple<TI, first_stage_first_stage::ACTOR_TYPE>;
    using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS<TYPE_POLICY, DEVICE::index_t>>;
    using OFF_POLICY_RUNNER_TYPE = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;
    OFF_POLICY_RUNNER_TYPE off_policy_runner;
    rlt::malloc(device, off_policy_runner);
    auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, 0);
    auto batch_group = rlt::get_group(device, data_file, "batch");
    load_dataset(device, batch_group, replay_buffer);
    static_assert(first_stage_second_stage::TD3_PARAMETERS::ACTOR_BATCH_SIZE == first_stage_second_stage::TD3_PARAMETERS::CRITIC_BATCH_SIZE, "ACTOR_BATCH_SIZE must be CRITIC_BATCH_SIZE");
    replay_buffer.position = first_stage_second_stage::TD3_PARAMETERS::ACTOR_BATCH_SIZE;

    constexpr DEVICE::index_t SEQUENCE_LENGTH = 1;
    using ACTOR_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<OFF_POLICY_RUNNER_SPEC, SEQUENCE_LENGTH, first_stage_second_stage::ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>;
    rlt::rl::components::off_policy_runner::SequentialBatch<ACTOR_BATCH_SPEC> actor_batch;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<rlt::rl::algorithms::td3::ActorTrainingBuffersSpecification<first_stage_second_stage::ActorCriticType::SPEC>> actor_training_buffers;
    rlt::malloc(device, actor_batch);
    rlt::malloc(device, actor_training_buffers);

    first_stage_second_stage::CRITIC_TYPE::Buffer<> critic_buffers[2];
    rlt::malloc(device, critic_buffers[0]);
    rlt::malloc(device, critic_buffers[1]);

    first_stage_second_stage::ACTOR_TYPE::Buffer<> actor_buffers[2];
    rlt::malloc(device, actor_buffers[0]);
    rlt::malloc(device, actor_buffers[1]);


    decltype(actor_critic.actor) pre_actor;
    rlt::malloc(device, pre_actor);
    rlt::copy(device, device, actor_critic.actor, pre_actor);
    rlt::reset_optimizer_state(device, actor_critic.actor_optimizer, actor_critic.actor);
    T mean_ratio = 0;
    T mean_ratio_grad = 0;
    T mean_ratio_adam = 0;
    TI num_updates = data_file.getGroup("actor_training").getNumberObjects();
    for(TI training_step_i = 0; training_step_i < num_updates; training_step_i++){
        decltype(actor_critic.actor) post_actor;
        rlt::malloc(device, post_actor);
        std::stringstream ss;
        ss << "actor_training/" << training_step_i;
        auto post_actor_group = rlt::get_group(device, data_file, ss.str());
        rlt::load(device, post_actor, post_actor_group);

//        DTYPE actor_1_loss = rlt::train_actor<AC_DEVICE, ActorCriticType::SPEC, ReplayBufferType::CAPACITY, typeof(rng), true>(device, actor_critic, replay_buffer, rng);
        rlt::gather_batch<DEVICE, OFF_POLICY_RUNNER_SPEC, ACTOR_BATCH_SPEC, decltype(rng), true>(device, off_policy_runner, actor_batch, rng);
        rlt::train_actor(device, actor_critic, actor_batch, actor_critic.actor_optimizer, actor_buffers[0], critic_buffers[0], actor_training_buffers, rng);

        rlt::reset_forward_state(device, pre_actor);
        rlt::reset_forward_state(device, post_actor);
        rlt::reset_forward_state(device, actor_critic.actor);
        decltype(actor_critic.actor) compare_actor;
        rlt::malloc(device, compare_actor);
        rlt::copy(device, device, actor_critic.actor, compare_actor);

        {
            rlt::utils::typing::remove_reference_t<decltype(actor_critic.critic_optimizers[0])> reset_optimizer;
            rlt::malloc(device, reset_optimizer);
            rlt::reset_optimizer_state(device, reset_optimizer, pre_actor);
            rlt::reset_optimizer_state(device, reset_optimizer, post_actor);
            rlt::reset_optimizer_state(device, reset_optimizer, compare_actor);
            rlt::free(device, reset_optimizer);
        }

        T pre_post_diff_per_weight = abs_diff(device, pre_actor, post_actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_target_per_weight = abs_diff(device, post_actor, compare_actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_ratio = pre_post_diff_per_weight/diff_target_per_weight;

        T pre_post_diff_grad_per_weight = abs_diff_grad(device, pre_actor, post_actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_target_grad_per_weight = abs_diff_grad(device, post_actor, actor_critic.actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_ratio_grad = pre_post_diff_grad_per_weight/diff_target_grad_per_weight;

        T pre_post_diff_adam_per_weight = abs_diff_adam(device, pre_actor, post_actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_target_adam_per_weight = abs_diff_adam(device, post_actor, compare_actor)/first_stage_second_stage::ActorCriticType::SPEC::ACTOR_TYPE::NUM_WEIGHTS;
        T diff_ratio_adam = pre_post_diff_adam_per_weight/diff_target_adam_per_weight;

        rlt::free(device, compare_actor);

        if(verbose){
            std:: cout << "    actor update" << std::endl;
//                std::cout << "        pre_post_diff_per_weight: " << pre_post_diff_per_weight << std::endl;
//                std::cout << "        diff_target_per_weight: " << diff_target_per_weight << std::endl;
            std::cout << "        update ratio     : " << diff_ratio << std::endl;

//                std::cout << "        pre_post_diff_grad_per_weight: " << pre_post_diff_grad_per_weight << std::endl;
//                std::cout << "        diff_target_grad_per_weight: " << diff_target_grad_per_weight << std::endl;
            std::cout << "        update ratio grad: " << diff_ratio_grad << std::endl;

//                std::cout << "        pre_post_diff_adam_per_weight: " << pre_post_diff_adam_per_weight << std::endl;
//                std::cout << "        diff_target_adam_per_weight: " << diff_target_adam_per_weight << std::endl;
            std::cout << "        update ratio adam: " << diff_ratio_adam << std::endl;
        }

        mean_ratio += diff_ratio;
        mean_ratio_grad += diff_ratio_grad;
        mean_ratio_adam += diff_ratio_adam;

//        ASSERT_LT(diff_target_per_weight, 1e-7);
//        actor_critic.critics[0] = post_critic;

    }
    mean_ratio /= num_updates;
    mean_ratio_grad /= num_updates;
    mean_ratio_adam /= num_updates;
    std::cout << "mean_ratio: " << mean_ratio << std::endl;
    std::cout << "mean_ratio_grad: " << mean_ratio_grad << std::endl;
    std::cout << "mean_ratio_adam: " << mean_ratio_adam << std::endl;
    ASSERT_GT(mean_ratio, 1e15); // TANH introduces a lot of inaccuracy
    ASSERT_GT(mean_ratio_grad, 1e15);
    ASSERT_GT(mean_ratio_adam, 1e15);
    rlt::free(device, replay_buffer);
}
