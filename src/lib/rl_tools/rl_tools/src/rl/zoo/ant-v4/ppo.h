#include "environment.h"

#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/loop/steps/extrack/config.h>
#include <rl_tools/rl/loop/steps/checkpoint/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/save_trajectories/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::ant_v4::ppo{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            static constexpr TI STEP_LIMIT = 600; // ~2.5M env steps

            static constexpr TI ACTOR_HIDDEN_DIM = 256;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
            static constexpr TI N_ENVIRONMENTS = 64;
            static constexpr TI BATCH_SIZE = 2048;
            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 64;
            static constexpr bool NORMALIZE_OBSERVATIONS = true;
            static constexpr bool NORMALIZE_OBSERVATIONS_CONTINUOUSLY = true;

            struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
                static constexpr TI N_EPOCHS = 4;
                static constexpr bool LEARN_ACTION_STD = true;
                static constexpr T INITIAL_ACTION_STD = 0.5;
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.0;
                static constexpr bool NORMALIZE_ADVANTAGE = false;
                static constexpr T GAMMA = 0.99;
                static constexpr bool ADAPTIVE_LEARNING_RATE = true;
                static constexpr T ADAPTIVE_LEARNING_RATE_POLICY_KL_THRESHOLD = 0.008;
            };
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequential, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
