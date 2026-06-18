#include <rl_tools/rl/environments/mujoco/ant/ant.h>

#include <rl_tools/nn_models/models.h>
#include <rl_tools/rl/algorithms/td3/td3.h>
#include <rl_tools/rl/components/off_policy_runner/off_policy_runner.h>

#include <rl_tools/utils/generic/typing.h>

namespace parameters_0{

    template<typename T, typename TI>
    struct environment{
        using ENVIRONMENT_SPEC = rlt::rl::environments::mujoco::ant::Specification<T, TI, rlt::rl::environments::mujoco::ant::DefaultParameters<T, TI>>;
        using ENVIRONMENT = rlt::rl::environments::mujoco::Ant<ENVIRONMENT_SPEC>;
    };

    template<typename T, typename TI, typename ENVIRONMENT>
    struct rl{
        struct ACTOR_CRITIC_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<T, TI>{
            static constexpr TI ACTOR_BATCH_SIZE = 256;
            static constexpr TI CRITIC_BATCH_SIZE = 256;
            static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
            static constexpr TI ACTOR_TRAINING_INTERVAL = 2;
            static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 2;
            static constexpr TI ACTOR_TARGET_UPDATE_INTERVAL = 2;
            static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.5;
            static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.2;
            static constexpr bool IGNORE_TERMINATION = false;
        };
        template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
        using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

        using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, ACTOR_CRITIC_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
        using ACTOR_CONFIG = rlt::nn_models::mlp::Configuration<T, TI, ENVIRONMENT::ACTION_DIM, 3, 256, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
        using ACTOR_MLP = rlt::nn_models::mlp::BindConfiguration<ACTOR_CONFIG>;

        using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
        using ACTOR_MODULE_CHAIN = Module<ACTOR_MLP>;
        using ACTOR_TYPE = rlt::nn_models::sequential::Build<ACTOR_CAPABILITY, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
        using ACTOR_TARGET_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;

        using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, ACTOR_CRITIC_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
        using CRITIC_CONFIG = rlt::nn_models::mlp::Configuration<T, TI, 1, 3, 256, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;
        using CRITIC_MLP = rlt::nn_models::mlp::BindConfiguration<CRITIC_CONFIG>;
        using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

        using MODULE_CHAIN = Module<CRITIC_MLP>;
        using CRITIC_TYPE = rlt::nn_models::sequential::Build<CRITIC_CAPABILITY, MODULE_CHAIN, CRITIC_INPUT_SHAPE>;
        using CRITIC_TARGET_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, CRITIC_INPUT_SHAPE>;

        using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<T, TI>;
        using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;

        using ACTOR_CRITIC_SPEC = rlt::rl::algorithms::td3::Specification<T, TI, ENVIRONMENT, ACTOR_TYPE, ACTOR_TARGET_TYPE, CRITIC_TYPE, CRITIC_TARGET_TYPE, OPTIMIZER, ACTOR_CRITIC_PARAMETERS>;
        using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<ACTOR_CRITIC_SPEC>;

        struct OFF_POLICY_RUNNER_PARAMETERS: rlt::rl::components::off_policy_runner::ParametersDefault<T, TI>{
            static constexpr TI N_ENVIRONMENTS = 1;
            static constexpr TI REPLAY_BUFFER_CAPACITY = 1000000;
            static constexpr TI EPISODE_STEP_LIMIT = 1000;
            static constexpr bool COLLECT_EPISODE_STATS = true;
            static constexpr TI EPISODE_STATS_BUFFER_SIZE = 1000;
            static constexpr T EXPLORATION_NOISE = 0.1;
        };
        using POLICIES = rlt::utils::Tuple<TI, ACTOR_TYPE>;
        using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<T, TI, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS>;

        static constexpr TI N_WARMUP_STEPS_CRITIC = 10000;
        static constexpr TI N_WARMUP_STEPS_ACTOR = 10000;
    };


}
