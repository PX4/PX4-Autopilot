#include <rl_tools/operations/cpu.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/environments/operations_cpu.h>
#include <rl_tools/rl/algorithms/td3/operations_cpu.h>

#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn_models/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/rl/utils/evaluation/operations_generic.h>
#include <rl_tools/utils/generic/memcpy.h>

#include "../../../utils/utils.h"
#include "../../../utils/nn_comparison_mlp.h"

#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY
#include <rl_tools/rl/environments/pendulum/ui.h>
#include <rl_tools/rl/utils/evaluation_visual.h>
#endif

#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_OUTPUT_PLOTS
#include "plot_policy_and_value_function.h"
#endif

#include <gtest/gtest.h>
#include <highfive/H5File.hpp>


std::string get_data_file_path(){
    std::string DATA_FILE_NAME = "model_second_stage.hdf5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    return DATA_FILE_PATH;
}
using T = double;
using DEVICE = rlt::devices::DefaultCPU;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
typedef rlt::rl::environments::pendulum::Specification<T, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<T>> PENDULUM_SPEC;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY
typedef rlt::rl::environments::pendulum::UI<T> UI;
#endif
ENVIRONMENT env;
ENVIRONMENT::Parameters env_parameters;

using AC_DEVICE = rlt::devices::DefaultCPU;

struct TD3ParametersCopyTraining: public rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, AC_DEVICE::index_t>{
    constexpr static typename AC_DEVICE::index_t CRITIC_BATCH_SIZE = 100;
    constexpr static typename AC_DEVICE::index_t ACTOR_BATCH_SIZE = 100;
};
template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;


using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, TD3ParametersCopyTraining::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
using ACTOR_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
using ACTOR = rlt::nn_models::mlp::BindConfiguration<ACTOR_NETWORK_SPEC>;
using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, TD3ParametersCopyTraining::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
using CRITIC_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;
using CRITIC = rlt::nn_models::mlp::BindConfiguration<CRITIC_NETWORK_SPEC>;

using NN_DEVICE = rlt::devices::DefaultCPU;
using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t, rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

//using ACTOR_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, ACTOR_CAPABILITY, ACTOR_INPUT_SHAPE>;
using ACTOR_MODULE_CHAIN = Module<ACTOR>;
using ACTOR_TYPE = rlt::nn_models::sequential::Build<ACTOR_CAPABILITY, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;

//using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, rlt::nn::capability::Forward<>, ACTOR_INPUT_SHAPE>;
using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;


using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
//using CRITIC_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, CRITIC_CAPABILITY, CRITIC_INPUT_SHAPE>;
using CRITIC_MODULE_CHAIN = Module<CRITIC>;
using CRITIC_TYPE = rlt::nn_models::sequential::Build<CRITIC_CAPABILITY, CRITIC_MODULE_CHAIN, CRITIC_INPUT_SHAPE>;

//using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, rlt::nn::capability::Forward<>, CRITIC_INPUT_SHAPE>;
using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, CRITIC_MODULE_CHAIN, CRITIC_INPUT_SHAPE>;

using TD3_SPEC = rlt::rl::algorithms::td3::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, ACTOR_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3ParametersCopyTraining>;
using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<TD3_SPEC>;


TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_SECOND_STAGE, TEST_LOADING_TRAINED_ACTOR) {
    constexpr bool verbose = false;
    AC_DEVICE device;
    NN_DEVICE nn_device;
    ActorCriticType actor_critic;
//    actor_critic.actor_optimizer.parameters = rlt::nn::optimizers::adam::default_parameters_torch<T>;
//    actor_critic.critic_optimizers[0].parameters = rlt::nn::optimizers::adam::default_parameters_torch<T>;
//    actor_critic.critic_optimizers[1].parameters = rlt::nn::optimizers::adam::default_parameters_torch<T>;

    ActorCriticType::SPEC::ACTOR_TYPE::Buffer<1> eval_buffers;
    rlt::malloc(device, actor_critic);
    rlt::malloc(device, eval_buffers);

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::rl::environments::DummyUI ui;

    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    TI step = data_file.getGroup("full_training").getGroup("steps").getNumberObjects()-1;
    // assert(step >= 0);
    auto step_group = data_file.getGroup("full_training").getGroup("steps").getGroup(std::to_string(step));
    rlt::persist::backends::hdf5::Group<> actor_group = {step_group.getGroup("actor")};
    rlt::load(device, actor_critic.actor.content, actor_group);
    using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, decltype(env), 100, 200>;
    rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
    rlt::evaluate(device, env, ui, actor_critic.actor, result, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
    std::cout << "mean return: " << result.returns_mean << std::endl;
}

//using ReplayBufferSpecCopyTraining = rlt::rl::components::replay_buffer::Specification<T, AC_DEVICE::index_t, 3, 1, 1000>;
using TI = AC_DEVICE::index_t;
struct OFF_POLICY_RUNNER_PARAMETERS{
    static constexpr TI N_ENVIRONMENTS = 1;
    static constexpr bool ASYMMETRIC_OBSERVATIONS = false;
    static constexpr TI REPLAY_BUFFER_CAPACITY = 1000;
    static constexpr bool STOCHASTIC_POLICY = false;
    static constexpr bool COLLECT_EPISODE_STATS = false;
    static constexpr TI EPISODE_STATS_BUFFER_SIZE = 0;
    static constexpr T EXPLORATION_NOISE = 0.1;
};
using POLICIES = rlt::utils::Tuple<TI, ACTOR_TYPE>;
using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS>;
using OFF_POLICY_RUNNER_TYPE = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;
using DEVICE = rlt::devices::DefaultCPU;
typedef OFF_POLICY_RUNNER_TYPE::REPLAY_BUFFER_TYPE ReplayBufferTypeCopyTraining;
constexpr TI BATCH_DIM = ENVIRONMENT::Observation::DIM * 2 + ENVIRONMENT::ACTION_DIM + 2;
template <typename DEVICE>
void load(DEVICE& device, ReplayBufferTypeCopyTraining& rb, std::vector<std::vector<T>> batch){
    for(TI i = 0; i < batch.size(); i++){
//        rlt::utils::memcpy(&rb.     rlt::get(observations, i, 0), &batch[i][0], ENVIRONMENT::Observation::DIM);
        rlt::assign(device, &batch[i][0], rb.observations, i, 0, 1, ENVIRONMENT::Observation::DIM);
//        rlt::utils::memcpy(&rb.          rlt::get(actions, i, 0), &batch[i][ENVIRONMENT::Observation::DIM], ENVIRONMENT::ACTION_DIM);
        rlt::assign(device, &batch[i][ENVIRONMENT::Observation::DIM], rb.actions, i, 0, 1, ENVIRONMENT::ACTION_DIM);
//        rlt::utils::memcpy(&rlt::get(rb.next_observations, i, 0), &batch[i][ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM], ENVIRONMENT::Observation::DIM);
        rlt::assign(device, &batch[i][ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM], rb.next_observations, i, 0, 1, ENVIRONMENT::Observation::DIM);
        rlt::set(rb.rewards, i, 0, batch[i][ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM + ENVIRONMENT::Observation::DIM]);
        rlt::set(rb.terminated, i, 0, batch[i][ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM + ENVIRONMENT::Observation::DIM + 1] == 1);
        rlt::set(rb.truncated, i, 0, batch[i][ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM + ENVIRONMENT::Observation::DIM + 2] == 1);
    }
    rb.position = batch.size();
}
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_SECOND_STAGE, FP_ACC) {
    for(TI i = 0; i < 1000; i++){
        std::normal_distribution<float> dist;
        auto rng = std::mt19937(0);
        float a = dist(rng) * 5e-3;
        float b = dist(rng) / 10;
        float aa = dist(rng);

        float c = a * b;
        c += aa;
        float d = c / b;
        d -= aa / b;
        float e = a - d;

//        std::cout << e << std::endl;
    }
    for(TI i = 0; i < 1000; i++){
        std::normal_distribution<double> dist;
        auto rng = std::mt19937(0);
        double a = dist(rng) * 5e-3;
        double b = dist(rng) / 10;
        double aa = dist(rng);

        double c = a * b;
        c += aa;
        double d = c / b;
        d -= aa / b;
        double e = a - d;

//        std::cout << e << std::endl;
    }
}
struct SequentialBatchParameters{
    static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = false;
    static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
    static constexpr bool RANDOM_SEQ_LENGTH = false;
    static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = false;
    static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.5;
};
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_MLP_SECOND_STAGE, TEST_COPY_TRAINING) {
#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY
    UI ui;
#endif
    constexpr bool verbose = true;
    AC_DEVICE device;
    NN_DEVICE nn_device;
    ActorCriticType actor_critic;
    ActorCriticType::SPEC::ACTOR_TYPE::Buffer<1> actor_eval_buffers;
    rlt::malloc(device, actor_critic);
    rlt::malloc(device, actor_eval_buffers);

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::init(device, actor_critic,rng);

    rlt::get_ref(device, actor_critic.actor_optimizer.parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[0].parameters, 0).epsilon_sqrt = 0;
    rlt::get_ref(device, actor_critic.critic_optimizers[1].parameters, 0).epsilon_sqrt = 0;


    rlt::rl::environments::DummyUI ui;



    auto data_file = HighFive::File(get_data_file_path(), HighFive::File::ReadOnly);
    rlt::persist::backends::hdf5::Group<> actor_group = {data_file.getGroup("actor")};
    rlt::load(device, actor_critic.actor.content, actor_group);
    rlt::persist::backends::hdf5::Group<> actor_target_group = {data_file.getGroup("actor_target")};
    rlt::load(device, actor_critic.actor_target.content, actor_target_group);
    rlt::persist::backends::hdf5::Group<> critic_1_group = {data_file.getGroup("critic_1")};
    rlt::load(device, actor_critic.critics[0].content, critic_1_group);
    rlt::persist::backends::hdf5::Group<> critic_target_1_group = {data_file.getGroup("critic_target_1")};
    rlt::load(device, actor_critic.critics_target[0].content, critic_target_1_group);
    rlt::persist::backends::hdf5::Group<> critic_2_group = {data_file.getGroup("critic_2")};
    rlt::load(device, actor_critic.critics[1].content, critic_2_group);
    rlt::persist::backends::hdf5::Group<> critic_target_2_group = {data_file.getGroup("critic_target_2")};
    rlt::load(device, actor_critic.critics_target[1].content, critic_target_2_group);

    OFF_POLICY_RUNNER_TYPE off_policy_runner;
    rlt::malloc(device, off_policy_runner);

    rlt::reset_optimizer_state(device, actor_critic.actor_optimizer     , actor_critic.actor   );
    rlt::reset_optimizer_state(device, actor_critic.critic_optimizers[0], actor_critic.critics[0]);
    rlt::reset_optimizer_state(device, actor_critic.critic_optimizers[1], actor_critic.critics[1]);
    T mean_ratio_critic = 0;
    T mean_ratio_critic_grad = 0;
    T mean_ratio_critic_adam = 0;
    T mean_ratio_actor = 0;
    T mean_ratio_actor_grad = 0;
    T mean_ratio_actor_adam = 0;
    T mean_ratio_critic_target = 0;
    auto full_training_group = data_file.getGroup("full_training");
    auto steps_group = full_training_group.getGroup("steps");
    TI num_steps = std::min(steps_group.getNumberObjects(), (typename DEVICE::index_t)1000);
    rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> pre_critic_1, pre_critic_2;
    rlt::malloc(device, pre_critic_1);
    rlt::malloc(device, pre_critic_2);
    rlt::copy(device, device, actor_critic.critics[0], pre_critic_1);
    rlt::copy(device, device, actor_critic.critics[1], pre_critic_2);
    decltype(actor_critic.actor) pre_actor;
    rlt::malloc(device, pre_actor);
    rlt::copy(device, device, actor_critic.actor, pre_actor);
    rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics_target[0])> pre_critic_1_target;
    rlt::malloc(device, pre_critic_1_target);
    rlt::copy(device, device, actor_critic.critics_target[0], pre_critic_1_target);

    using CRITIC_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE, SequentialBatchParameters>;
    rlt::rl::components::off_policy_runner::SequentialBatch<CRITIC_BATCH_SPEC> critic_batch;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<ActorCriticType::SPEC>> critic_training_buffers;
    CRITIC_TYPE::Buffer<> critic_buffers[2];
    rlt::malloc(device, critic_batch);
    rlt::malloc(device, critic_training_buffers);
    rlt::malloc(device, critic_buffers[0]);
    rlt::malloc(device, critic_buffers[1]);

    using ACTOR_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE, SequentialBatchParameters>;
    rlt::rl::components::off_policy_runner::SequentialBatch<ACTOR_BATCH_SPEC> actor_batch;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<rlt::rl::algorithms::td3::ActorTrainingBuffersSpecification<ActorCriticType::SPEC>> actor_training_buffers;
    ACTOR_TYPE::Buffer<> actor_buffers[2];
    rlt::malloc(device, actor_batch);
    rlt::malloc(device, actor_training_buffers);
    rlt::malloc(device, actor_buffers[0]);
    rlt::malloc(device, actor_buffers[1]);

    for(TI step_i = 0; step_i < num_steps; step_i++){
        if(verbose){
            std::cout << "step_i: " << step_i << std::endl;
        }
        auto _step_group = steps_group.getGroup(std::to_string(step_i));
        rlt::persist::backends::hdf5::Group<> step_group = {_step_group};
        if(_step_group.exist("critics_batch")){
            std::vector<std::vector<T>> batch;
            _step_group.getDataSet("critics_batch").read(batch);
            assert(batch.size() == ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);

//            step_group.getDataSet("target_next_action_noise").read(critic_training_buffers.target_next_action_noise.data);
            {
                auto target_next_action_noise_matrix_view = rlt::matrix_view(device, critic_training_buffers.target_next_action_noise);
                rlt::load(device, target_next_action_noise_matrix_view, step_group, "target_next_action_noise");
            }

            auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, 0);
            load(device, replay_buffer, batch);
//            if (step_i == 0 && step_group.exist("pre_critic1")){
//                decltype(actor_critic.critics[0]) pre_critic_1_step;
//                rlt::malloc(device, pre_critic_1_step);
//                rlt::load(device, pre_critic_1_step, step_group.getGroup("pre_critic1"));
//                rlt::reset_forward_state(device, pre_critic_1_step);
//                rlt::reset_forward_state(device, actor_critic.critics[0]);
//                T pre_current_diff = abs_diff(device, pre_critic_1_step, actor_critic.critics[0]);
//                ASSERT_EQ(pre_current_diff, 0);
//                rlt::free(device, pre_critic_1_step);
//            }

            rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics[0])> post_critic_1, post_critic_2;// = actor_critic.critics[0];
            rlt::malloc(device, post_critic_1);
            rlt::malloc(device, post_critic_2);
            auto critic1_group = rlt::get_group(device, step_group, "critic1");
            auto critic2_group = rlt::get_group(device, step_group, "critic2");
            rlt::load(device, post_critic_1.content, critic1_group);
            rlt::load(device, post_critic_2.content, critic2_group);


            rlt::gather_batch<DEVICE, OFF_POLICY_RUNNER_SPEC, CRITIC_BATCH_SPEC, decltype(rng), true>(device, off_policy_runner, critic_batch, rng);
            rlt::train_critic(device, actor_critic, actor_critic.critics[0], critic_batch, actor_critic.critic_optimizers[0], actor_buffers[0], actor_buffers[0], critic_buffers[0], critic_buffers[0], critic_training_buffers, rng);


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

            T pre_post_diff_per_weight = abs_diff(device, pre_critic_1.content, post_critic_1.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_per_weight = abs_diff(device, post_critic_1.content, compare_critic.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_ratio = pre_post_diff_per_weight/diff_target_per_weight;

            T pre_post_diff_grad_per_weight = abs_diff_grad(device, pre_critic_1.content, post_critic_1.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_grad_per_weight = abs_diff_grad(device, post_critic_1.content, actor_critic.critics[0].content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_ratio_grad = pre_post_diff_grad_per_weight/diff_target_grad_per_weight;

            T pre_post_diff_adam_per_weight = abs_diff_adam(device, pre_critic_1.content, post_critic_1.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_adam_per_weight = abs_diff_adam(device, post_critic_1.content, compare_critic.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_ratio_adam = pre_post_diff_adam_per_weight/diff_target_adam_per_weight;

            rlt::free(device, compare_critic);

            if(verbose){
                std:: cout << "    critic update" << std::endl;
                std::cout << "        update ratio     : " << diff_ratio << std::endl;
                std::cout << "        update ratio grad: " << diff_ratio_grad << std::endl;
                std::cout << "        update ratio adam: " << diff_ratio_adam << std::endl;
            }
            if(diff_ratio < 1e10){
//                std::cout << "something wrong here" << std::endl;
            }

            switch(step_i){
                case 0: {
                    ASSERT_GT(diff_ratio, 1e6);
                    ASSERT_GT(diff_ratio_grad, 1e6);
                    ASSERT_GT(diff_ratio_adam, 1e6);
                }
                    break;
            }

            mean_ratio_critic += diff_ratio;
            mean_ratio_critic_grad += diff_ratio_grad;
            mean_ratio_critic_adam += diff_ratio_adam;

            {
//                step_group.getDataSet("target_next_action_noise").read(critic_training_buffers.target_next_action_noise.data);
                auto target_next_action_noise_matrix_view = rlt::matrix_view(device, critic_training_buffers.target_next_action_noise);
                rlt::load(device, target_next_action_noise_matrix_view, step_group, "target_next_action_noise");

                rlt::gather_batch<DEVICE, OFF_POLICY_RUNNER_SPEC, CRITIC_BATCH_SPEC, decltype(rng), true>(device, off_policy_runner, critic_batch, rng);
                rlt::train_critic(device, actor_critic, actor_critic.critics[1], critic_batch, actor_critic.critic_optimizers[1], actor_buffers[0], actor_buffers[0], critic_buffers[0], critic_buffers[0], critic_training_buffers, rng);
            }

//            if(false){//(step_i % 100 == 0){
//                T diff = 0;
//                for(TI batch_sample_i = 0; batch_sample_i < ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE; batch_sample_i++){
//                    T input[ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM + ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM];
//                    rlt::utils::memcpy(input, &rlt::get(replay_buffer.observations, batch_sample_i, 0), ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM);
//                    rlt::utils::memcpy(&input[ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM], &rlt::get(replay_buffer.actions, batch_sample_i, 0), ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM);
//                    using input_layout = rlt::matrix::layouts::RowMajorAlignment<DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM + ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM, 1>;
//                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM + ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM, layout>> input_matrix = {input};
//                    T current_value;
//                    using current_value_layout = rlt::matrix::layouts::RowMajorAlignment<DEVICE::index_t, 1, 1, 1>;
//                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1, current_value_layout>> current_value_matrix = {&current_value};
//                    rlt::evaluate(device, actor_critic.critics[0], input_matrix, current_value_matrix);
////                    T desired_value;
////                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> desired_value_matrix = {&desired_value};
////                    rlt::evaluate(device, post_critic_1, input_matrix, desired_value_matrix);
////                    diff += (current_value - desired_value) * (current_value - desired_value) / ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
//                }
////                std::cout << "value mse: " << diff << std::endl;
//            }
            rlt::free(device, post_critic_1);
        }

        if(_step_group.exist("actor_batch")){
            std::vector<std::vector<T>> batch;
            _step_group.getDataSet("actor_batch").read(batch);
            assert(batch.size() == ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE);
            auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, 0);
            load(device, replay_buffer, batch);

            decltype(actor_critic.actor) post_actor;
            rlt::malloc(device, post_actor);
            auto actor_group = rlt::get_group(device, step_group, "actor");
            rlt::load(device, post_actor.content, actor_group);

            decltype(actor_critic.actor) pre_actor_loaded;
            rlt::malloc(device, pre_actor_loaded);
            auto pre_actor_group = rlt::get_group(device, step_group, "pre_actor");
            rlt::load(device, pre_actor_loaded.content, pre_actor_group);
            rlt::reset_forward_state(device, pre_actor_loaded);
            rlt::reset_forward_state(device, actor_critic.actor);
            T pre_current_diff = abs_diff(device, pre_actor_loaded.content, actor_critic.actor.content);
            if(step_i == 0){
                ASSERT_EQ(pre_current_diff, 0);
            }


            {
                rlt::gather_batch<DEVICE, OFF_POLICY_RUNNER_SPEC, ACTOR_BATCH_SPEC, decltype(rng), true>(device, off_policy_runner, actor_batch, rng);
                rlt::train_actor(device, actor_critic, actor_batch, actor_critic.actor_optimizer, actor_buffers[0], critic_buffers[0], actor_training_buffers, rng);
            }
//            T actor_loss = rlt::train_actor<AC_DEVICE, ActorCriticType::SPEC, decltype(replay_buffer)::CAPACITY, typeof(rng), true>(device, actor_critic, replay_buffer, rng);

//            if(true){//(step_i % 100 == 1){
//                T diff = 0;
//                for(TI batch_sample_i = 0; batch_sample_i < ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE; batch_sample_i++){
//                    T current_action[ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM];
//                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM>> current_action_matrix = {current_action};
//                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM>> observation_matrix = {&replay_buffer.observations.data[batch_sample_i]};
//                    rlt::evaluate(device, actor_critic.actor, observation_matrix, current_action_matrix);
//                    T desired_action[ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM];
//                    rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM>> desired_action_matrix = {desired_action};
//                    rlt::evaluate(device, post_actor, observation_matrix, desired_action_matrix);
//                    diff += rlt::nn::loss_functions::mse::evaluate(device, current_action_matrix, desired_action_matrix, T(1)/ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE);
//                }
////                std::cout << "action mse: " << diff << std::endl;
//            }

            rlt::reset_forward_state(device, pre_actor);
            rlt::reset_forward_state(device, post_actor);
            rlt::reset_forward_state(device, actor_critic.actor);
            decltype(actor_critic.actor) compare_actor;
            rlt::malloc(device, compare_actor);
            rlt::copy(device, device, actor_critic.actor, compare_actor);

            {
                rlt::utils::typing::remove_reference_t<decltype(actor_critic.critic_optimizers[0])> reset_optimizer;
                rlt::malloc(device, reset_optimizer);
                rlt::init(device, reset_optimizer);
                rlt::reset_optimizer_state(device, reset_optimizer, pre_actor);
                rlt::reset_optimizer_state(device, reset_optimizer, post_actor);
                rlt::reset_optimizer_state(device, reset_optimizer, compare_actor);
                rlt::free(device, reset_optimizer);
            }

            T pre_post_diff_per_weight = abs_diff(device, pre_actor.content, post_actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_per_weight = abs_diff(device, post_actor.content, compare_actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_ratio = pre_post_diff_per_weight/diff_target_per_weight;

            T pre_post_diff_grad_per_weight = abs_diff_grad(device, pre_actor.content, post_actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_grad_per_weight = abs_diff_grad(device, post_actor.content, actor_critic.actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_ratio_grad = pre_post_diff_grad_per_weight/diff_target_grad_per_weight;

            T pre_post_diff_adam_per_weight = abs_diff_adam(device, pre_actor.content, post_actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
            T diff_target_adam_per_weight = abs_diff_adam(device, post_actor.content, compare_actor.content)/ActorCriticType::SPEC::ACTOR_TYPE::CONTENT::NUM_WEIGHTS;
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

            switch(step_i){
                case 0: {
                    ASSERT_GT(diff_ratio, 1e6);
                    ASSERT_GT(diff_ratio_grad, 1e6);
                    ASSERT_GT(diff_ratio_adam, 1e6);
                }
                break;
            }

            mean_ratio_actor += diff_ratio;
            mean_ratio_actor_grad += diff_ratio_grad;
            mean_ratio_actor_adam += diff_ratio_adam;

            rlt::copy(device, device, actor_critic.actor, pre_actor);
            rlt::free(device, post_actor);
            rlt::free(device, pre_actor_loaded);
        }
        if(_step_group.exist("critic1_target")){
            if(verbose){
                std:: cout << "    target update" << std::endl;
            }
            if (step_i == 0){
                rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics_target[0])> pre_critic_1_target_step;
                rlt::malloc(device, pre_critic_1_target_step);
                auto critic1_target_group = rlt::get_group(device, step_group, "critic1_target");
                rlt::load(device, pre_critic_1_target_step.content, critic1_target_group);
                T pre_current_diff = abs_diff(device, pre_critic_1_target_step.content, actor_critic.critics_target[0].content);
                ASSERT_EQ(pre_current_diff, 0);
                rlt::free(device, pre_critic_1_target_step);
            }
            else{
                if (step_i >= ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE){

                    rlt::utils::typing::remove_reference_t<decltype(actor_critic.critics_target[0])> post_critic_1_target;
                    rlt::malloc(device, post_critic_1_target);
                    auto critic1_target_group = rlt::get_group(device, step_group, "critic1_target");
                    rlt::load(device, post_critic_1_target.content, critic1_target_group);

                    rlt::update_critic_targets(device, actor_critic);
                    rlt::update_actor_target(device, actor_critic);

                    T pre_post_diff_per_weight = abs_diff(device, pre_critic_1_target.content, post_critic_1_target.content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
                    T diff_target_per_weight = abs_diff(device, post_critic_1_target.content, actor_critic.critics_target[0].content)/ActorCriticType::SPEC::CRITIC_TYPE::CONTENT::NUM_WEIGHTS;
                    T diff_ratio = pre_post_diff_per_weight/diff_target_per_weight;

                    if(verbose){
                        std::cout << "    critic target update" << std::endl;
//                        std::cout << "        pre_post_diff_per_weight: " << pre_post_diff_per_weight << std::endl;
//                        std::cout << "        diff_target_per_weight: " << diff_target_per_weight << std::endl;
                        std::cout << "        update ratio     : " << diff_ratio << std::endl;
                    }

                    switch(step_i){
                        case 0: {
                            ASSERT_GT(diff_ratio, 1e6);
                        }
                            break;
                    }

                    mean_ratio_critic_target += diff_ratio;

                    rlt::copy(device, device, actor_critic.critics_target[0], pre_critic_1_target);

//                    if(true){//(step_i % 100 == 0){
//                        T diff = 0;
//                        for(TI batch_sample_i = 0; batch_sample_i < ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE; batch_sample_i++){
//                            T input[ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM + ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM];
//                            rlt::utils::memcpy(input, &replay_buffer.observations.data[batch_sample_i*ENVIRONMENT::Observation::DIM], ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM);
//                            rlt::utils::memcpy(&input[ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM], &replay_buffer.actions.data[batch_sample_i*ENVIRONMENT::ACTION_DIM], ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM);
//                            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ActorCriticType::SPEC::ENVIRONMENT::Observation::DIM + ActorCriticType::SPEC::ENVIRONMENT::ACTION_DIM>> input_matrix = {input};
//                            T current_value;
//                            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> current_value_matrix = {&current_value};
//                            rlt::evaluate(device, actor_critic.critics_target[0], input_matrix, current_value_matrix);
//                            T desired_value;
//                            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, 1>> desired_value_matrix = {&desired_value};
//                            rlt::evaluate(device, post_critic_1_target, input_matrix, desired_value_matrix);
//                            diff += (current_value - desired_value) * (current_value - desired_value) / ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
//                        }
////                        std::cout << "value mse: " << diff << std::endl;
//                    }
                    rlt::free(device, post_critic_1_target);
                }
            }

        }
        if(step_i % 100 == 0){
            if(!verbose){
                std::cout << "step_i: " << step_i << std::endl;
            }
            using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, decltype(env), 100, 200>;
            rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
            rlt::evaluate(device, env, ui, actor_critic.actor, result, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_OUTPUT_PLOTS
            plot_policy_and_value_function<T, ENVIRONMENT, ActorCriticType::ACTOR_TYPE, ActorCriticType::CRITIC_TYPE>(actor_critic.actor, actor_critic.critics[0], std::string("second_stage"), step_i);
#endif
#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY
            if(mean_return > -400){
                while(true){
                    ENVIRONMENT::State initial_state;
                    rlt::sample_initial_state(env, initial_state, rng);
                    rlt::evaluate_visual<ENVIRONMENT, UI, decltype(actor_critic.actor), 100, 3>(env, ui, actor_critic.actor, initial_state);
                }
            }
#endif
        }
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
        if(step_i >= 10){
            break;
        }
#endif

    }
    mean_ratio_critic /= num_steps;
    mean_ratio_critic_grad /= num_steps;
    mean_ratio_critic_adam /= num_steps;
    mean_ratio_actor /= num_steps;
    mean_ratio_actor_grad /= num_steps;
    mean_ratio_actor_adam /= num_steps;
    mean_ratio_critic_target /= num_steps;
    std::cout << "mean_ratio_critic: " << mean_ratio_critic << std::endl;
    std::cout << "mean_ratio_critic_grad: " << mean_ratio_critic_grad << std::endl;
    std::cout << "mean_ratio_critic_adam: " << mean_ratio_critic_adam << std::endl;
    std::cout << "mean_ratio_actor: " << mean_ratio_actor << std::endl;
    std::cout << "mean_ratio_actor_grad: " << mean_ratio_actor_grad << std::endl;
    std::cout << "mean_ratio_actor_adam: " << mean_ratio_actor_adam << std::endl;
    std::cout << "mean_ratio_critic_target: " << mean_ratio_critic_target << std::endl;

#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_GT(mean_ratio_critic, 1e12);
    ASSERT_GT(mean_ratio_critic_grad, 1e13);
    ASSERT_GT(mean_ratio_critic_adam, 1e12);
    ASSERT_GT(mean_ratio_actor, 1e12);
    ASSERT_GT(mean_ratio_actor_grad, 1e12);
    ASSERT_GT(mean_ratio_actor_adam, 1e12);
    ASSERT_GT(mean_ratio_critic_target, 1e11);
#endif

    rlt::free(device, critic_batch);
    rlt::free(device, critic_training_buffers);
    rlt::free(device, actor_batch);
    rlt::free(device, actor_training_buffers);
    rlt::free(device, off_policy_runner);
}
