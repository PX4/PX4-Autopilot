// Group 1
#include <rl_tools/operations/cuda/group_1.h>
#include <rl_tools/operations/cpu/group_1.h>
#include <rl_tools/operations/cpu_mkl/group_1.h>

// Group 2
#include <rl_tools/operations/cuda/group_2.h>
#include <rl_tools/operations/cpu/group_2.h>
#include <rl_tools/operations/cpu_mkl/group_2.h>

// Group 3
#include <rl_tools/operations/cuda/group_3.h>
#include <rl_tools/operations/cpu/group_3.h>
#include <rl_tools/operations/cpu_mkl/group_3.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_cuda.h>
#include <rl_tools/nn/operations_cpu_mkl.h>
#include <rl_tools/nn/operations_cuda.h>
#include <rl_tools/nn/loss_functions/mse/operations_cuda.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/rl/components/replay_buffer/operations_cpu.h>
#include <rl_tools/rl/components/replay_buffer/persist.h>
#include <rl_tools/rl/components/off_policy_runner/operations_cpu.h>

#include <rl_tools/rl/environments/pendulum/operations_cpu.h>

#include <rl_tools/rl/components/off_policy_runner/operations_cuda.h>
#include <rl_tools/rl/algorithms/td3/operations_cuda.h>
#include <rl_tools/rl/algorithms/td3/operations_cpu.h>

#include "../components/replay_buffer.h"


#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

class RL_TOOLS_RL_CUDA : public ::testing::Test {
public:
    using DEVICE_CPU = rlt::devices::DefaultCPU_MKL;
    using DEVICE_GPU = rlt::devices::DefaultCUDA;
    using NN_DEVICE = DEVICE_CPU;
    using DTYPE = double;
    static constexpr DEVICE_CPU::index_t CAPACITY = 20000;
    static constexpr DEVICE_CPU::index_t BATCH_SIZE = 256;
    static constexpr DTYPE EPSILON = (rlt::utils::typing::is_same_v<DTYPE, float> ? 1e-5 : 1e-10);// * BATCH_SIZE;
//    using REPLAY_BUFFER_SPEC = rlt::rl::components::replay_buffer::Specification<DTYPE, DEVICE_CPU::index_t, OBSERVATION_DIM, ACTION_DIM, CAPACITY>;
//    using REPLAY_BUFFER = rlt::rl::components::ReplayBuffer<REPLAY_BUFFER_SPEC>;
    using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<DTYPE, DEVICE_CPU::index_t, rlt::rl::environments::pendulum::DefaultParameters<DTYPE>>;
    using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
    using T = DTYPE;
    using TI = DEVICE_CPU::index_t;
    struct OFF_POLICY_RUNNER_PARAMETERS: rlt::rl::components::off_policy_runner::ParametersDefault<T, TI>{
        static constexpr TI REPLAY_BUFFER_CAPACITY = CAPACITY;
        static constexpr TI EPISODE_STEP_LIMIT = 100;
        static constexpr bool STOCHASTIC_POLICY = false;
        static constexpr bool COLLECT_EPISODE_STATS = false;
        static constexpr TI EPISODE_STATS_BUFFER_SIZE = 0;
    };
    using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<DTYPE, DEVICE_CPU::index_t, ENVIRONMENT, OFF_POLICY_RUNNER_PARAMETERS>;
    using OFF_POLICY_RUNNER_TYPE = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;
    using BATCH_SPEC = rlt::rl::components::off_policy_runner::BatchSpecification<OFF_POLICY_RUNNER_SPEC, BATCH_SIZE>;
    using BATCH_TYPE = rlt::rl::components::off_policy_runner::Batch<BATCH_SPEC>;
    struct TD3_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<DTYPE, NN_DEVICE::index_t>{
        static constexpr typename NN_DEVICE::index_t ACTOR_BATCH_SIZE = BATCH_SIZE;
        static constexpr typename NN_DEVICE::index_t CRITIC_BATCH_SIZE = BATCH_SIZE;
    };
    using ACTOR_SPEC = rlt::nn_models::mlp::Specification<DTYPE, NN_DEVICE::index_t, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
    using CRITIC_SPEC = rlt::nn_models::mlp::Specification<DTYPE, NN_DEVICE::index_t, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;
    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<DTYPE, NN_DEVICE::index_t>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, TD3_PARAMETERS::ACTOR_BATCH_SIZE>;
    using ACTOR_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_CAPABILITY, ACTOR_SPEC>;
    using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<rlt::nn::capability::Forward, ACTOR_SPEC>;
    using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, TD3_PARAMETERS::CRITIC_BATCH_SIZE>;
    using CRITIC_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_CAPABILITY, CRITIC_SPEC>;
    using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<rlt::nn::capability::Forward, CRITIC_SPEC>;
    using ACTOR_CRITIC_SPEC = rlt::rl::algorithms::td3::Specification<DTYPE, NN_DEVICE::index_t, ENVIRONMENT, ACTOR_NETWORK_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_NETWORK_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3_PARAMETERS>;
    using ACTOR_CRITIC_TYPE = rlt::rl::algorithms::td3::ActorCritic<ACTOR_CRITIC_SPEC>;
    using ACTOR_BUFFERS = ACTOR_NETWORK_TYPE::template Buffer<ACTOR_CRITIC_SPEC::PARAMETERS::ACTOR_BATCH_SIZE>; //rlt::nn_models::mlp::NeuralNetworkBuffers<rlt::nn_models::mlp::NeuralNetworkBuffersSpecification<ACTOR_SPEC, ACTOR_CRITIC_SPEC::PARAMETERS::ACTOR_BATCH_SIZE>>;
    using CRITIC_BUFFERS = CRITIC_NETWORK_TYPE::template Buffer<ACTOR_CRITIC_SPEC::PARAMETERS::CRITIC_BATCH_SIZE>; //= rlt::nn_models::mlp::NeuralNetworkBuffers<rlt::nn_models::mlp::NeuralNetworkBuffersSpecification<CRITIC_SPEC, ACTOR_CRITIC_SPEC::PARAMETERS::CRITIC_BATCH_SIZE>>;
    DEVICE_CPU device_cpu;
    DEVICE_GPU device_gpu;
    OFF_POLICY_RUNNER_TYPE off_policy_runner_cpu;
    OFF_POLICY_RUNNER_TYPE off_policy_runner_cpu_2;
    OFF_POLICY_RUNNER_TYPE off_policy_runner_gpu_cpu;
    OFF_POLICY_RUNNER_TYPE* off_policy_runner_gpu_struct;
    BATCH_TYPE batch_cpu, batch_cpu_2;
    BATCH_TYPE batch_gpu;
    BATCH_TYPE* batch_gpu_struct;
    ACTOR_CRITIC_TYPE actor_critic_cpu, actor_critic_cpu_2;
    ACTOR_CRITIC_TYPE actor_critic_gpu;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<ACTOR_CRITIC_SPEC> critic_training_buffers_cpu;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<ACTOR_CRITIC_SPEC> critic_training_buffers_cpu_2;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<ACTOR_CRITIC_SPEC> critic_training_buffers_gpu;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<ACTOR_CRITIC_SPEC> actor_training_buffers_cpu;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<ACTOR_CRITIC_SPEC> actor_training_buffers_cpu_2;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<ACTOR_CRITIC_SPEC> actor_training_buffers_gpu;
    ACTOR_BUFFERS actor_buffers_cpu;
    ACTOR_BUFFERS actor_buffers_cpu_2;
    ACTOR_BUFFERS actor_buffers_gpu;
    CRITIC_BUFFERS critic_buffers_cpu;
    CRITIC_BUFFERS critic_buffers_cpu_2;
    CRITIC_BUFFERS critic_buffers_gpu;
    rlt::Matrix<rlt::matrix::Specification<DTYPE, TI, CRITIC_NETWORK_TYPE::SPEC::BATCH_SIZE, 1>> d_critic_output_cpu;
    rlt::Matrix<rlt::matrix::Specification<DTYPE, TI, CRITIC_NETWORK_TYPE::SPEC::BATCH_SIZE, 1>> d_critic_output_gpu;
protected:
    void SetUp() override {
        rlt::init(device_gpu);
        auto rng_cpu = rlt::random::default_engine(device_cpu);
        auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());
        // alloc
        rlt::malloc(device_cpu, off_policy_runner_cpu);
        rlt::malloc(device_cpu, off_policy_runner_cpu_2);
        rlt::malloc(device_gpu, off_policy_runner_gpu_cpu);
        rlt::malloc(device_cpu, batch_cpu);
        rlt::malloc(device_cpu, batch_cpu_2);
        rlt::malloc(device_gpu, batch_gpu);
        cudaMalloc(&off_policy_runner_gpu_struct, sizeof(OFF_POLICY_RUNNER_TYPE));
        cudaMalloc(&batch_gpu_struct, sizeof(BATCH_TYPE));
        rlt::malloc(device_cpu, actor_critic_cpu);
        rlt::malloc(device_cpu, actor_critic_cpu_2);
        rlt::malloc(device_gpu, actor_critic_gpu);
        rlt::malloc(device_cpu, critic_training_buffers_cpu);
        rlt::malloc(device_cpu, critic_training_buffers_cpu_2);
        rlt::malloc(device_gpu, critic_training_buffers_gpu);
        rlt::malloc(device_cpu, actor_buffers_cpu);
        rlt::malloc(device_cpu, actor_buffers_cpu_2);
        rlt::malloc(device_gpu, actor_buffers_gpu);
        rlt::malloc(device_cpu, critic_buffers_cpu);
        rlt::malloc(device_cpu, critic_buffers_cpu_2);
        rlt::malloc(device_gpu, critic_buffers_gpu);
        rlt::malloc(device_cpu, actor_training_buffers_cpu);
        rlt::malloc(device_cpu, actor_training_buffers_cpu_2);
        rlt::malloc(device_gpu, actor_training_buffers_gpu);
        rlt::malloc(device_cpu, d_critic_output_cpu);
        rlt::malloc(device_gpu, d_critic_output_gpu);

        // init
        for(DEVICE_CPU::index_t rb_i = 0; rb_i < OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS; rb_i++) {
            rlt::test::rl::components::replay_buffer::sample(device_cpu, off_policy_runner_cpu.replay_buffers[rb_i], rng_cpu);
            rlt::copy(device_cpu, device_gpu, off_policy_runner_cpu.replay_buffers[rb_i], off_policy_runner_gpu_cpu.replay_buffers[rb_i]);
        }
        rlt::init(device_cpu, actor_critic_cpu, rng_cpu);

        // copy
        rlt::check_status(device_gpu);
        cudaMemcpy(off_policy_runner_gpu_struct, &off_policy_runner_gpu_cpu, sizeof(OFF_POLICY_RUNNER_TYPE), cudaMemcpyHostToDevice);
        rlt::check_status(device_gpu);
        cudaMemcpy(batch_gpu_struct, &batch_gpu, sizeof(BATCH_TYPE), cudaMemcpyHostToDevice);
        rlt::check_status(device_gpu);
        rlt::copy(device_cpu, device_gpu, actor_critic_cpu, actor_critic_gpu);
    }

    void TearDown() override {
        rlt::free(device_cpu, off_policy_runner_cpu);
        rlt::free(device_cpu, off_policy_runner_cpu_2);
        rlt::free(device_gpu, off_policy_runner_gpu_cpu);
        rlt::free(device_cpu, batch_cpu);
        rlt::free(device_cpu, batch_cpu_2);
        rlt::free(device_gpu, batch_gpu);
        cudaFree(off_policy_runner_gpu_struct);
        cudaFree(batch_gpu_struct);
        rlt::free(device_cpu, actor_critic_cpu);
        rlt::free(device_cpu, actor_critic_cpu_2);
        rlt::free(device_gpu, actor_critic_gpu);
        rlt::free(device_cpu, critic_training_buffers_cpu);
        rlt::free(device_cpu, critic_training_buffers_cpu_2);
        rlt::free(device_gpu, critic_training_buffers_gpu);
        rlt::free(device_cpu, actor_buffers_cpu);
        rlt::free(device_cpu, actor_buffers_cpu_2);
        rlt::free(device_gpu, actor_buffers_gpu);
        rlt::free(device_cpu, critic_buffers_cpu);
        rlt::free(device_cpu, critic_buffers_cpu_2);
        rlt::free(device_gpu, critic_buffers_gpu);
        rlt::free(device_cpu, actor_training_buffers_cpu);
        rlt::free(device_cpu, actor_training_buffers_cpu_2);
        rlt::free(device_gpu, actor_training_buffers_gpu);
    }
};

TEST_F(RL_TOOLS_RL_CUDA, VIEW_COPY_PROBLEM) {

    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    rlt::randn(device_cpu, batch_cpu.observations_actions_next_observations, rng_cpu);
    rlt::set_all(device_cpu, batch_cpu_2.observations_actions_next_observations, 0);
    rlt::copy(device_cpu, device_gpu, batch_cpu.next_observations, batch_gpu.next_observations);
    rlt::copy(device_gpu, device_cpu, batch_gpu.next_observations, batch_cpu_2.next_observations);

    auto abs_diff_next_observations = rlt::abs_diff(device_cpu, batch_cpu_2.next_observations, batch_cpu.next_observations);
    ASSERT_LT(abs_diff_next_observations, EPSILON);
    ASSERT_LT(rlt::sum(device_cpu, batch_cpu_2.observations), EPSILON);
    ASSERT_LT(rlt::sum(device_cpu, batch_cpu_2.actions), EPSILON);
}

TEST_F(RL_TOOLS_RL_CUDA, GATHER_BATCH) {

    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());
    for(DEVICE_CPU::index_t rb_i = 0; rb_i < OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS; rb_i++) {
        rlt::copy(device_gpu, device_cpu, off_policy_runner_gpu_cpu.replay_buffers[rb_i], off_policy_runner_cpu_2.replay_buffers[rb_i]);
        auto abs_diff = rlt::abs_diff(device_cpu, off_policy_runner_cpu.replay_buffers[rb_i], off_policy_runner_cpu_2.replay_buffers[rb_i]);
        ASSERT_FLOAT_EQ(abs_diff, 0);
    }

    rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
    rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);

    auto batch_observation_view = rlt::view<DEVICE_CPU, typename decltype(off_policy_runner_cpu.replay_buffers[0].observations)::SPEC, BATCH_SIZE, BATCH_TYPE::OBSERVATION_DIM>(device_cpu, off_policy_runner_cpu.replay_buffers[0].observations, 0, 0);
    rlt::print(device_cpu, batch_observation_view);
    std::cout << "BATCH" << std::endl;
    rlt::print(device_cpu, batch_cpu.observations);
    std::cout << "BATCH GPU" << std::endl;
    rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu_2);
    rlt::print(device_cpu, batch_cpu_2.observations);

    auto abs_diff_batch = rlt::abs_diff(device_cpu, batch_cpu.observations, batch_cpu_2.observations);
    abs_diff_batch += rlt::abs_diff(device_cpu, batch_cpu.actions, batch_cpu_2.actions);
    abs_diff_batch += rlt::abs_diff(device_cpu, batch_cpu.next_observations, batch_cpu_2.next_observations);
    abs_diff_batch += rlt::abs_diff(device_cpu, batch_cpu.rewards, batch_cpu_2.rewards);
    abs_diff_batch += rlt::abs_diff(device_cpu, batch_cpu.terminated, batch_cpu_2.terminated);
    abs_diff_batch += rlt::abs_diff(device_cpu, batch_cpu.truncated, batch_cpu_2.truncated);
    ASSERT_FLOAT_EQ(abs_diff_batch, 0);
}
TEST_F(RL_TOOLS_RL_CUDA, TRAIN_CRITIC_STEP_BY_STEP) {
    constexpr DEVICE_CPU::index_t N_STEPS = 5;

    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    auto sample_batch = [&](bool deterministic){
        rng_gpu = rlt::random::next(DEVICE_GPU::SPEC::RANDOM(), rng_gpu);
        if(deterministic){
            rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
            rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);

            // action noise from cpu
            rlt::target_action_noise(device_cpu, actor_critic_cpu, critic_training_buffers_cpu.target_next_action_noise, rng_cpu);
            rlt::copy(device_cpu, device_gpu, critic_training_buffers_cpu.target_next_action_noise, critic_training_buffers_gpu.target_next_action_noise);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu_2.target_next_action_noise);
            auto abs_diff_target_next_action_noise = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            std::cout << "abs_diff_target_next_action_noise: " << abs_diff_target_next_action_noise << std::endl;
            ASSERT_FLOAT_EQ(abs_diff_target_next_action_noise, 0);
        }
        else{
            rlt::gather_batch(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
            rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu);

            // action noise from gpu
            rlt::target_action_noise(device_gpu, actor_critic_gpu, critic_training_buffers_gpu.target_next_action_noise, rng_gpu);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std = rlt::std(device_cpu, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std_diff = std::abs(action_noise_std - ACTOR_CRITIC_SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_STD);
            std::cout << "action_noise_std_diff: " << action_noise_std_diff << std::endl;
            ASSERT_LT(action_noise_std_diff, 0.05);
        }

//    rlt::target_action_noise(device_gpu, actor_critic_gpu, critic_training_buffers_gpu.target_next_action_noise, rng_gpu);

        static_assert(BATCH_SPEC::BATCH_SIZE == ACTOR_BUFFERS::BATCH_SIZE);

        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
        auto abs_diff_actor_critic = rlt::abs_diff(device_cpu, actor_critic_cpu_2.actor_target, actor_critic_cpu.actor_target);
        std::cout << "abs_diff_actor_critic: " << abs_diff_actor_critic << std::endl;
        ASSERT_FLOAT_EQ(abs_diff_actor_critic, 0);

        rlt::evaluate(device_cpu, actor_critic_cpu.actor.input_layer, batch_cpu.observations, actor_buffers_cpu.tick, actor_buffers_cpu.layer_buffer, rng_gpu);
        rlt::evaluate(device_gpu, actor_critic_gpu.actor.input_layer, batch_gpu.observations, actor_buffers_gpu.tick,  actor_buffers_gpu.layer_buffer, rng_gpu);
        rlt::copy(device_gpu, device_cpu, actor_buffers_gpu, actor_buffers_cpu_2);
        auto abs_diff_tick = rlt::abs_diff(device_cpu, actor_buffers_cpu_2.tick, actor_buffers_cpu.tick);
        std::cout << "abs_diff_tick: " << abs_diff_tick << std::endl;
        ASSERT_LT(abs_diff_tick, EPSILON);

        rlt::evaluate(device_cpu, actor_critic_cpu.actor_target, batch_cpu.next_observations, critic_training_buffers_cpu.next_actions, actor_buffers_cpu, rng_gpu);
        rlt::evaluate(device_gpu, actor_critic_gpu.actor_target, batch_gpu.next_observations, critic_training_buffers_gpu.next_actions, actor_buffers_gpu, rng_gpu);

        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        auto abs_diff_next_actions = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.next_actions, critic_training_buffers_cpu.next_actions);
        std::cout << "abs_diff_next_actions: " << abs_diff_next_actions << std::endl;
        ASSERT_LT(abs_diff_next_actions, EPSILON);

        rlt::noisy_next_actions(device_cpu, critic_training_buffers_cpu);
        rlt::noisy_next_actions(device_gpu, critic_training_buffers_gpu);
        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        rlt::check_status(device_gpu);
        auto abs_diff_noisy_next_actions = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.next_actions, critic_training_buffers_cpu.next_actions);
        std::cout << "abs_diff_noisy_next_actions: " << abs_diff_noisy_next_actions << std::endl;
        ASSERT_LT(abs_diff_noisy_next_actions, EPSILON);

        rlt::copy(device_cpu, device_cpu, batch_cpu.next_observations, critic_training_buffers_cpu.next_observations);
        rlt::copy(device_gpu, device_gpu, batch_gpu.next_observations, critic_training_buffers_gpu.next_observations);
        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        auto abs_diff_next_state_action_value_input = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.next_state_action_value_input, critic_training_buffers_cpu.next_state_action_value_input);
        std::cout << "abs_diff_next_state_action_value_input: " << abs_diff_next_state_action_value_input << std::endl;
        ASSERT_LT(abs_diff_next_state_action_value_input, EPSILON);

        rlt::evaluate(device_cpu, actor_critic_cpu.critic_target_1, critic_training_buffers_cpu.next_state_action_value_input, critic_training_buffers_cpu.next_state_action_value_critic_1, critic_buffers_cpu, rng_cpu);
        rlt::evaluate(device_cpu, actor_critic_cpu.critic_target_2, critic_training_buffers_cpu.next_state_action_value_input, critic_training_buffers_cpu.next_state_action_value_critic_2, critic_buffers_cpu, rng_gpu);

        rlt::evaluate(device_gpu, actor_critic_gpu.critic_target_1, critic_training_buffers_gpu.next_state_action_value_input, critic_training_buffers_gpu.next_state_action_value_critic_1, critic_buffers_gpu, rng_cpu);
        rlt::evaluate(device_gpu, actor_critic_gpu.critic_target_2, critic_training_buffers_gpu.next_state_action_value_input, critic_training_buffers_gpu.next_state_action_value_critic_2, critic_buffers_gpu, rng_gpu);

        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        auto abs_diff_next_state_action_value_critic_1 = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.next_state_action_value_critic_1, critic_training_buffers_cpu.next_state_action_value_critic_1);
        auto abs_diff_next_state_action_value_critic_2 = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.next_state_action_value_critic_2, critic_training_buffers_cpu.next_state_action_value_critic_2);
        std::cout << "abs_diff_next_state_action_value_critic_1: " << abs_diff_next_state_action_value_critic_1 << std::endl;
        std::cout << "abs_diff_next_state_action_value_critic_2: " << abs_diff_next_state_action_value_critic_2 << std::endl;
        ASSERT_LT(abs_diff_next_state_action_value_critic_1, EPSILON);
        ASSERT_LT(abs_diff_next_state_action_value_critic_2, EPSILON);

        rlt::target_action_values(device_cpu, actor_critic_cpu, batch_cpu, critic_training_buffers_cpu);
        rlt::target_action_values(device_gpu, actor_critic_gpu, batch_gpu, critic_training_buffers_gpu);
        rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu, critic_training_buffers_cpu_2);
        auto abs_diff_target_action_value = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.target_action_value, critic_training_buffers_cpu.target_action_value);
        std::cout << "abs_diff_target_action_value: " << abs_diff_target_action_value << std::endl;
        ASSERT_LT(abs_diff_target_action_value, EPSILON);
    };

    sample_batch(true);

    for(typename DEVICE_CPU::index_t i = 0; i < N_STEPS; i++){
        typename DEVICE_CPU::index_t critic_i = i % 2;
        sample_batch(false);
        auto& critic_cpu = critic_i == 0 ? actor_critic_cpu.critic_1 : actor_critic_cpu.critic_2;
        auto& critic_gpu = critic_i == 0 ? actor_critic_gpu.critic_1 : actor_critic_gpu.critic_2;
        auto& critic_cpu_2 = critic_i == 0 ? actor_critic_cpu_2.critic_1 : actor_critic_cpu_2.critic_2;

        rlt::zero_gradient(device_cpu, critic_cpu);
        rlt::zero_gradient(device_gpu, critic_gpu);

//        forward_backward_mse(device_cpu, critic_cpu, batch_cpu.observations_and_actions, critic_training_buffers_cpu.target_action_value, critic_buffers_cpu);
        {
            rlt::forward(device_cpu, critic_cpu, batch_cpu.observations_and_actions, critic_buffers_cpu, rng_cpu);
            rlt::nn::loss_functions::mse::gradient(device_cpu, output(critic_cpu), critic_training_buffers_cpu.target_action_value, d_critic_output_cpu);
            rlt::backward(device_cpu, critic_cpu, batch_cpu.observations_and_actions, d_critic_output_cpu, critic_buffers_cpu);
        }
//        forward_backward_mse(device_gpu, critic_gpu, batch_gpu.observations_and_actions, critic_training_buffers_gpu.target_action_value, critic_buffers_gpu);
        {
            rlt::forward(device_gpu, critic_gpu, batch_gpu.observations_and_actions, critic_buffers_gpu, rng_gpu);
            rlt::nn::loss_functions::mse::gradient(device_gpu, output(critic_gpu), critic_training_buffers_gpu.target_action_value, d_critic_output_gpu);
            rlt::backward(device_gpu, critic_gpu, batch_gpu.observations_and_actions, d_critic_output_gpu, critic_buffers_gpu);
        }
        rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);

        auto abs_diff_critic = rlt::abs_diff(device_cpu, critic_cpu, critic_cpu_2);
        std::cout << "abs_diff_critic: " << abs_diff_critic << std::endl;
        ASSERT_LT(abs_diff_critic, EPSILON);

        rlt::step(device_cpu, actor_critic_cpu.critic_optimizers[0], critic_cpu);
        rlt::step(device_gpu, actor_critic_gpu.critic_optimizers[0], critic_gpu);
        rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
        auto abs_diff_critic_after_update = rlt::abs_diff(device_cpu, critic_cpu, critic_cpu_2);
        std::cout << "abs_diff_critic_after_update: " << abs_diff_critic_after_update << std::endl;
        ASSERT_LT(abs_diff_critic_after_update, EPSILON);

        if(i % 5 == 0){
            rlt::update_critic_targets(device_cpu, actor_critic_cpu);
            rlt::update_critic_targets(device_gpu, actor_critic_gpu);
            rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
            auto abs_diff_critic_target_1 = rlt::abs_diff(device_cpu, actor_critic_cpu.critic_target_1, actor_critic_cpu_2.critic_target_1);
            auto abs_diff_critic_target_2 = rlt::abs_diff(device_cpu, actor_critic_cpu.critic_target_2, actor_critic_cpu_2.critic_target_2);
            std::cout << "abs_diff_critic_target_1: " << abs_diff_critic_target_1 << std::endl;
            std::cout << "abs_diff_critic_target_2: " << abs_diff_critic_target_2 << std::endl;
            ASSERT_LT(abs_diff_critic_target_1, EPSILON);
            ASSERT_LT(abs_diff_critic_target_2, EPSILON);
        }
    }


}

TEST_F(RL_TOOLS_RL_CUDA, TRAIN_CRITIC_CORRECTNESS) {
    constexpr DEVICE_CPU::index_t N_STEPS = 50;

    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    auto sample_batch = [&](bool deterministic){
        rng_gpu = rlt::random::next(DEVICE_GPU::SPEC::RANDOM(), rng_gpu);
        if(deterministic){
            rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
            rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);

            // action noise from cpu
            rlt::target_action_noise(device_cpu, actor_critic_cpu, critic_training_buffers_cpu.target_next_action_noise, rng_cpu);
            rlt::copy(device_cpu, device_gpu, critic_training_buffers_cpu.target_next_action_noise, critic_training_buffers_gpu.target_next_action_noise);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu_2.target_next_action_noise);
            auto abs_diff_target_next_action_noise = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            std::cout << "abs_diff_target_next_action_noise: " << abs_diff_target_next_action_noise << std::endl;
            ASSERT_FLOAT_EQ(abs_diff_target_next_action_noise, 0);
        }
        else{
            rlt::gather_batch(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
            rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu);

            // action noise from gpu
            rlt::target_action_noise(device_gpu, actor_critic_gpu, critic_training_buffers_gpu.target_next_action_noise, rng_gpu);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std = rlt::std(device_cpu, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std_diff = std::abs(action_noise_std - ACTOR_CRITIC_SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_STD);
            std::cout << "action_noise_std_diff: " << action_noise_std_diff << std::endl;
            ASSERT_LT(action_noise_std_diff, 0.05);
        }
    };

    sample_batch(true);

    for(typename DEVICE_CPU::index_t i = 0; i < N_STEPS; i++){
        typename DEVICE_CPU::index_t critic_i = i % 2;
        sample_batch(false);
        auto& critic_cpu = critic_i == 0 ? actor_critic_cpu.critic_1 : actor_critic_cpu.critic_2;
        auto& critic_gpu = critic_i == 0 ? actor_critic_gpu.critic_1 : actor_critic_gpu.critic_2;
        auto& critic_cpu_2 = critic_i == 0 ? actor_critic_cpu_2.critic_1 : actor_critic_cpu_2.critic_2;

        rlt::train_critic(device_cpu, actor_critic_cpu, critic_cpu, batch_cpu, actor_critic_cpu.critic_optimizers[0], actor_buffers_cpu, critic_buffers_cpu, critic_training_buffers_cpu, rng_cpu);
        rlt::train_critic(device_gpu, actor_critic_gpu, critic_gpu, batch_gpu, actor_critic_gpu.critic_optimizers[0], actor_buffers_gpu, critic_buffers_gpu, critic_training_buffers_gpu, rng_gpu);

        rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
        auto abs_diff_critic_after_update = rlt::abs_diff(device_cpu, critic_cpu, critic_cpu_2);
        std::cout << "abs_diff_critic_after_update: " << abs_diff_critic_after_update << std::endl;
        ASSERT_LT(abs_diff_critic_after_update, EPSILON);

        if(i % 5 == 0){
            rlt::update_critic_targets(device_cpu, actor_critic_cpu);
            rlt::update_critic_targets(device_gpu, actor_critic_gpu);
            rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
            auto abs_diff_critic_target_1 = rlt::abs_diff(device_cpu, actor_critic_cpu.critic_target_1, actor_critic_cpu_2.critic_target_1);
            auto abs_diff_critic_target_2 = rlt::abs_diff(device_cpu, actor_critic_cpu.critic_target_2, actor_critic_cpu_2.critic_target_2);
            std::cout << "abs_diff_critic_target_1: " << abs_diff_critic_target_1 << std::endl;
            std::cout << "abs_diff_critic_target_2: " << abs_diff_critic_target_2 << std::endl;
            ASSERT_LT(abs_diff_critic_target_1, EPSILON);
            ASSERT_LT(abs_diff_critic_target_2, EPSILON);
        }
    }
}

TEST_F(RL_TOOLS_RL_CUDA, TRAIN_CRITIC_PERFORMANCE) {
    using DEVICE_MKL = rlt::devices::DefaultCPU_MKL;
    DEVICE_MKL device_mkl;
    constexpr DEVICE_CPU::index_t N_STEPS = 10000;

    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    auto sample_batch = [&](bool deterministic){
        rng_gpu = rlt::random::next(DEVICE_GPU::SPEC::RANDOM(), rng_gpu);
        if(deterministic){
            rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
            rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);

            // action noise from cpu
            rlt::target_action_noise(device_cpu, actor_critic_cpu, critic_training_buffers_cpu.target_next_action_noise, rng_cpu);
            rlt::copy(device_cpu, device_gpu, critic_training_buffers_cpu.target_next_action_noise, critic_training_buffers_gpu.target_next_action_noise);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu_2.target_next_action_noise);
            auto abs_diff_target_next_action_noise = rlt::abs_diff(device_cpu, critic_training_buffers_cpu_2.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            std::cout << "abs_diff_target_next_action_noise: " << abs_diff_target_next_action_noise << std::endl;
            ASSERT_FLOAT_EQ(abs_diff_target_next_action_noise, 0);
        }
        else{
            rlt::gather_batch(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
            rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu);

            // action noise from gpu
            rlt::target_action_noise(device_gpu, actor_critic_gpu, critic_training_buffers_gpu.target_next_action_noise, rng_gpu);
            rlt::copy(device_gpu, device_cpu, critic_training_buffers_gpu.target_next_action_noise, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std = rlt::std(device_cpu, critic_training_buffers_cpu.target_next_action_noise);
            auto action_noise_std_diff = std::abs(action_noise_std - ACTOR_CRITIC_SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_STD);
            std::cout << "action_noise_std_diff: " << action_noise_std_diff << std::endl;
            ASSERT_LT(action_noise_std_diff, 0.05);
        }
    };

    sample_batch(true);

    {
        auto& critic_cpu = actor_critic_cpu.critic_1;
        auto start = std::chrono::high_resolution_clock::now();
        for(typename DEVICE_CPU::index_t i = 0; i < N_STEPS; i++){
            rlt::train_critic(device_mkl, actor_critic_cpu, critic_cpu, batch_cpu, actor_critic_cpu.critic_optimizers[0], actor_buffers_cpu, critic_buffers_cpu, critic_training_buffers_cpu, rng_cpu);
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "CPU train_critic: " << duration.count()/N_STEPS << " microseconds" << std::endl;
    }
    {
        auto& critic_gpu = actor_critic_gpu.critic_1;
        cudaDeviceSynchronize();
        auto start = std::chrono::high_resolution_clock::now();
        for(typename DEVICE_CPU::index_t i = 0; i < N_STEPS; i++){
            rlt::train_critic(device_gpu, actor_critic_gpu, critic_gpu, batch_gpu, actor_critic_gpu.critic_optimizers[0], actor_buffers_gpu, critic_buffers_gpu, critic_training_buffers_gpu, rng_gpu);
        }
        cudaDeviceSynchronize();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "GPU train_critic: " << duration.count()/N_STEPS << " microseconds" << std::endl;
    }
}

TEST_F(RL_TOOLS_RL_CUDA, TRAIN_ACTOR_CORRECTNESS) {
    constexpr DEVICE_CPU::index_t N_STEPS = 50;
    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    auto sample_batch = [&](bool deterministic){
        rng_gpu = rlt::random::next(DEVICE_GPU::SPEC::RANDOM(), rng_gpu);
        if(deterministic){
            rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
            rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
        }
        else{
            rlt::gather_batch(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
            rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu);
        }
    };
    for(typename DEVICE_CPU::index_t step_i = 0; step_i < N_STEPS; step_i++){
        sample_batch(false);

        rlt::train_actor(device_cpu, actor_critic_cpu, batch_cpu, actor_critic_cpu.actor_optimizer, actor_buffers_cpu, critic_buffers_cpu, actor_training_buffers_cpu, rng_cpu);
        rlt::train_actor(device_gpu, actor_critic_gpu, batch_gpu, actor_critic_gpu.actor_optimizer, actor_buffers_gpu, critic_buffers_gpu, actor_training_buffers_gpu, rng_gpu);

        rlt::copy(device_gpu, device_cpu, actor_critic_gpu, actor_critic_cpu_2);
        auto abs_diff_actor_after_update = rlt::abs_diff(device_cpu, actor_critic_cpu.actor, actor_critic_cpu_2.actor);
        std::cout << "abs_diff_actor_after_update: " << abs_diff_actor_after_update << std::endl;
        ASSERT_LT(abs_diff_actor_after_update, EPSILON);
    }

}

TEST_F(RL_TOOLS_RL_CUDA, TRAIN_ACTOR_PERFORMANCE) {
    constexpr DEVICE_CPU::index_t N_STEPS = 10000;
    auto rng_cpu = rlt::random::default_engine(device_cpu);
    auto rng_gpu = rlt::random::default_engine(DEVICE_GPU::SPEC::RANDOM());

    auto sample_batch = [&](bool deterministic){
        rng_gpu = rlt::random::next(DEVICE_GPU::SPEC::RANDOM(), rng_gpu);
        if(deterministic){
            rlt::gather_batch<DEVICE_CPU, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_cpu), true>(device_cpu, off_policy_runner_cpu, batch_cpu, rng_cpu);
            rlt::gather_batch<typename DEVICE_GPU::SPEC, OFF_POLICY_RUNNER_SPEC, BATCH_SPEC, decltype(rng_gpu), true>(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
        }
        else{
            rlt::gather_batch(device_gpu, off_policy_runner_gpu_struct, batch_gpu, rng_gpu);
            rlt::copy(device_gpu, device_cpu, batch_gpu, batch_cpu);
        }
    };
    sample_batch(false);

    auto start = std::chrono::high_resolution_clock::now();
    for(typename DEVICE_CPU::index_t step_i = 0; step_i < N_STEPS; step_i++){
        rlt::train_actor(device_cpu, actor_critic_cpu, batch_cpu, actor_critic_cpu.actor_optimizer, actor_buffers_cpu, critic_buffers_cpu, actor_training_buffers_cpu, rng_cpu);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "CPU time: " << duration.count()/N_STEPS << " microseconds" << std::endl;

    cudaDeviceSynchronize();
    start = std::chrono::high_resolution_clock::now();
    for(typename DEVICE_CPU::index_t step_i = 0; step_i < N_STEPS; step_i++){
        rlt::train_actor(device_gpu, actor_critic_gpu, batch_gpu, actor_critic_gpu.actor_optimizer, actor_buffers_gpu, critic_buffers_gpu, actor_training_buffers_gpu, rng_gpu);
    }
    cudaDeviceSynchronize();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "GPU time: " << duration.count()/N_STEPS << " microseconds" << std::endl;
}
