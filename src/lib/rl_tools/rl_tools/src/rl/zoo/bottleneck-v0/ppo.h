#include "environment.h"

#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::bottleneck_v0::ppo{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            static constexpr TI STEP_LIMIT = 5000; // 1024 * 4 * 74 ~ 300k steps

            static constexpr TI ACTOR_HIDDEN_DIM = 64;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI CRITIC_HIDDEN_DIM = 128;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
            static constexpr TI N_ENVIRONMENTS = 128;
            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 128;
            static constexpr TI BATCH_SIZE = 1024;
            struct OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 1e-3;
            };
            static constexpr bool NORMALIZE_OBSERVATIONS = true;
            struct PPO_PARAMETERS: rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
                static constexpr T GAMMA = 0.98;
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;
                static constexpr TI N_EPOCHS = 1;
                static constexpr bool IGNORE_TERMINATION = true;
//                static constexpr bool ADAPTIVE_LEARNING_RATE = true;
            };
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent, DYNAMIC_ALLOCATION>;
        template <typename BASE>
        struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{
//            static constexpr TI EPISODE_STEP_LIMIT = 20 / ENVIRONMENT_PARAMETERS::DT;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
