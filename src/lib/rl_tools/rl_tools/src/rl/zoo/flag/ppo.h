#include "environment.h"
#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::flag::ppo{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            static constexpr TI N_ENVIRONMENTS = 16;
            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 256;
            static constexpr TI BATCH_SIZE = N_ENVIRONMENTS*ON_POLICY_RUNNER_STEPS_PER_ENV;
            static constexpr TI TOTAL_STEP_LIMIT = 10000000;
            static constexpr TI ACTOR_HIDDEN_DIM = 128;
            static constexpr TI CRITIC_HIDDEN_DIM = 128;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI STEP_LIMIT = TOTAL_STEP_LIMIT/(ON_POLICY_RUNNER_STEPS_PER_ENV * N_ENVIRONMENTS) + 1;
            static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
            struct OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 0.0003;
            };
            static constexpr bool NORMALIZE_OBSERVATIONS = true;
            struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.0;
                static constexpr TI N_EPOCHS = 1;
                static constexpr T GAMMA = 0.995;
                static constexpr T LAMBDA = 0.95;
                static constexpr T INITIAL_ACTION_STD = 1.0;
            };
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequential, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
