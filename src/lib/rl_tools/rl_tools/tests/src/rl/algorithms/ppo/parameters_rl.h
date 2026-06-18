#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/rl/algorithms/ppo/ppo.h>
#include <rl_tools/rl/components/on_policy_runner/on_policy_runner.h>
namespace parameters_0{
    using namespace rlt;
    template <typename T, typename TI>
    struct environment{
        using ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<T, TI>;
        using ENVIRONMENT = rlt::rl::environments::Pendulum<ENVIRONMENT_SPEC>;
    };
    template <typename TYPE_POLICY, typename TI, typename ENVIRONMENT>
    struct rl{
        static constexpr TI BATCH_SIZE = 64;

        using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
        using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_PARAMETERS>;
        using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

        template <typename CAPABILITY>
        struct Actor{
            using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
            using ACTOR_SPEC = nn_models::mlp::Configuration<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::ActivationFunction::TANH, nn::activation_functions::IDENTITY>;
            using ACTOR = nn_models::mlp_unconditional_stddev::BindConfiguration<ACTOR_SPEC>;

            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
            using MODULE_CHAIN = Module<ACTOR>;

            using MODEL = nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
        };
//        using ACTOR_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::OPTIMIZER_PARAMETERS>;
//        using CRITIC_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::OPTIMIZER_PARAMETERS>;
//        using ACTOR_OPTIMIZER = nn::optimizers::Adam<ACTOR_OPTIMIZER_SPEC>;
//        using CRITIC_OPTIMIZER = nn::optimizers::Adam<CRITIC_OPTIMIZER_SPEC>;

        using ACTOR_TYPE = typename Actor<CAPABILITY_ADAM>::MODEL;
//        using ACTOR_TYPE_INFERENCE = typename Actor<nn::capability::Forward>::MODEL;
//        using ACTOR_TYPE = rlt::nn_models::mlp_unconditional_stddev::NeuralNetwork<CAPABILITY_ADAM, ACTOR_SPEC>;
        using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
        using CRITIC_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 1, 3, 64, rlt::nn::activation_functions::ActivationFunction::TANH, rlt::nn::activation_functions::IDENTITY>;
        using CRITIC = rlt::nn_models::mlp::BindConfiguration<CRITIC_SPEC>;
        template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
        using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
        using MODULE_CHAIN = Module<CRITIC>;

        using CRITIC_TYPE = nn_models::sequential::Build<CAPABILITY_ADAM, MODULE_CHAIN, CRITIC_INPUT_SHAPE>;


        struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
            static constexpr TI N_EPOCHS = 1;
        };
        using PPO_SPEC = rlt::rl::algorithms::ppo::Specification<TYPE_POLICY, TI, ENVIRONMENT, ACTOR_TYPE, CRITIC_TYPE, PPO_PARAMETERS>;
        using PPO_TYPE = rlt::rl::algorithms::PPO<PPO_SPEC>;
        using PPO_BUFFERS_TYPE = rlt::rl::algorithms::ppo::Buffers<rlt::rl::algorithms::ppo::BufferSpecification<PPO_SPEC>>;

        static constexpr TI ON_POLICY_RUNNER_STEP_LIMIT = 200;
        static constexpr TI N_ENVIRONMENTS = 10;
        using ON_POLICY_RUNNER_SPEC = rlt::rl::components::on_policy_runner::Specification<TYPE_POLICY, TI, ENVIRONMENT, N_ENVIRONMENTS, ON_POLICY_RUNNER_STEP_LIMIT>;
        using ON_POLICY_RUNNER_TYPE = rlt::rl::components::OnPolicyRunner<ON_POLICY_RUNNER_SPEC>;
        static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 200;
        using ON_POLICY_RUNNER_DATASET_SPEC = rlt::rl::components::on_policy_runner::DatasetSpecification<ON_POLICY_RUNNER_SPEC, ON_POLICY_RUNNER_STEPS_PER_ENV>;
        using ON_POLICY_RUNNER_DATASET_TYPE = rlt::rl::components::on_policy_runner::Dataset<ON_POLICY_RUNNER_DATASET_SPEC>;

        using ACTOR_EVAL_TYPE = typename ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS>;
        using ACTOR_EVAL_BUFFERS = typename ACTOR_EVAL_TYPE::template Buffer<>;
        using ACTOR_BUFFERS = typename ACTOR_TYPE::template Buffer<>;
        using CRITIC_BUFFERS = typename CRITIC_TYPE::template Buffer<>;
        using CRITIC_EVAL_TYPE_ALL = typename CRITIC_TYPE::template CHANGE_BATCH_SIZE<TI, ON_POLICY_RUNNER_DATASET_SPEC::STEPS_TOTAL_ALL>;
        using CRITIC_BUFFERS_ALL = typename CRITIC_EVAL_TYPE_ALL::template Buffer<>;
    };
}
