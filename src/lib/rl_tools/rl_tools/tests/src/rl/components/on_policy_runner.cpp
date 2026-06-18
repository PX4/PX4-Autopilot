#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/pendulum/pendulum.h>
#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/rl/components/on_policy_runner/on_policy_runner.h>
#include <rl_tools/rl/components/on_policy_runner/operations_generic.h>
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/rl/components/on_policy_runner/persist.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include <gtest/gtest.h>

using DEVICE = rlt::devices::DefaultCPU;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
using ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<T, TI>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<ENVIRONMENT_SPEC>;
constexpr TI BATCH_SIZE = 1;

template <typename CAPABILITY>
struct Actor{
    using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
    using ACTOR_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::ActivationFunction::TANH, rlt::nn::activation_functions::IDENTITY>;
    using ACTOR = rlt::nn_models::mlp_unconditional_stddev::BindConfiguration<ACTOR_SPEC>;
    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
    using MODULE_CHAIN = Module<ACTOR>;

    using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
};

TEST(RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER, TEST){

    constexpr TI N_ENVIRONMENTS = 3;
    using ON_POLICY_RUNNER_SPEC = rlt::rl::components::on_policy_runner::Specification<TYPE_POLICY, TI, ENVIRONMENT, N_ENVIRONMENTS>;
    using ON_POLICY_RUNNER = rlt::rl::components::OnPolicyRunner<ON_POLICY_RUNNER_SPEC>;


    DEVICE device;
    ON_POLICY_RUNNER runner;
    rlt::malloc(device, runner);
    ENVIRONMENT envs[ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS];
    ENVIRONMENT::Parameters parameters[ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS];
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 199);
    rlt::init(device, runner, envs, parameters, rng);

//    using ACTOR_SPEC = rlt::nn_models::mlp::Specification<T, TI, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::TANH>;
    using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
////    using ACTOR_TYPE = rlt::nn_models::mlp_unconditional_stddev::NeuralNetwork<CAPABILITY_ADAM, ACTOR_SPEC>;
    using ACTOR_TYPE = typename Actor<ACTOR_CAPABILITY>::MODEL;
    using ACTOR_ROLLOUT_TYPE = typename ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS>;
    using ACTOR_BUFFERS = typename ACTOR_ROLLOUT_TYPE::template Buffer<>;


    constexpr TI STEPS_PER_ENV = 1000;
    using DATASET_SPEC = rlt::rl::components::on_policy_runner::DatasetSpecification<ON_POLICY_RUNNER_SPEC, STEPS_PER_ENV>;
    using DATASET = rlt::rl::components::on_policy_runner::Dataset<DATASET_SPEC>;

    ACTOR_TYPE actor;
    ACTOR_BUFFERS actor_buffers;
    DATASET dataset;
    rlt::malloc(device, actor);
    rlt::malloc(device, actor_buffers);
    rlt::malloc(device, dataset);
    rlt::init_weights(device, actor, rng);
    rlt::set_all(device, dataset.data, 0);


    rlt::collect(device, dataset, runner, actor, actor_buffers, rng);
    rlt::print(device, dataset.data);
    rlt::collect(device, dataset, runner, actor, actor_buffers, rng);
    rlt::print(device, dataset.data);
    rlt::collect(device, dataset, runner, actor, actor_buffers, rng);
    rlt::print(device, dataset.data);
    ENVIRONMENT::State states[ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS];
    ENVIRONMENT::Parameters env_parameters[ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS];
    for(TI env_i = 0; env_i < ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS; env_i++){
        states[env_i] = get(runner.states, 0, env_i);
        parameters[env_i] = get(runner.env_parameters, 0, env_i);
    }
    rlt::collect(device, dataset, runner, actor, actor_buffers, rng);
    for(TI env_i = 0; env_i < ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS; env_i++){
        for(TI step_i = 0; step_i < DATASET_SPEC::STEPS_PER_ENV; step_i++){
            TI pos = step_i * ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS + env_i;
            {
                rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM>> observation;
                rlt::malloc(device, observation);
                rlt::observe(device, get(runner.environments, 0, env_i), parameters[env_i], states[env_i], typename ENVIRONMENT::Observation{}, observation, rng);
                auto observation_runner = rlt::view<DEVICE, decltype(dataset.observations)::SPEC, 1, ENVIRONMENT::Observation::DIM>(device, dataset.observations, pos, 0);
                auto abs_diff = rlt::abs_diff(device, observation, observation_runner);
                if(!get(dataset.truncated, pos, 0)){
//                    ASSERT_FLOAT_EQ(abs_diff, 0);
                }
                rlt::free(device, observation);
            }
            typename ENVIRONMENT::State next_state;
            auto action = rlt::view<DEVICE, decltype(dataset.actions)::SPEC, 1, ENVIRONMENT::ACTION_DIM>(device, dataset.actions, pos, 0);
            step(device, get(runner.environments, 0, env_i), parameters[env_i], states[env_i], action, next_state, rng);
            states[env_i] = next_state;
        }
    }
    std::string FILE_PATH = "test_rl_components_on_policy_runner_dataset.h5";
    {
        auto file = HighFive::File(FILE_PATH, HighFive::File::Overwrite);
        rlt::persist::backends::hdf5::Group<> dataset_group = {file.createGroup("dataset")};
        rlt::save(device, dataset, dataset_group);
    }

    {
        auto file = HighFive::File(FILE_PATH, HighFive::File::ReadOnly);
        DATASET loaded;
        rlt::malloc(device, loaded);
        rlt::persist::backends::hdf5::Group<> dataset_group = {file.getGroup("dataset")};
        rlt::load(device, loaded, dataset_group);
        auto abs_diff = rlt::abs_diff(device, loaded.data, dataset.data);
        ASSERT_FLOAT_EQ(0, abs_diff);
    }



}