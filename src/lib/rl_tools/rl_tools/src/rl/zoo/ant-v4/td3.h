#include "environment.h"

#include <rl_tools/rl/algorithms/td3/loop/core/config.h>
#include <rl_tools/rl/loop/steps/extrack/config.h>
#include <rl_tools/rl/loop/steps/checkpoint/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/save_trajectories/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>

namespace rl_tools::rl::zoo::ant_v4::td3{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct TD3_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, TI>{
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

            static constexpr T EXPLORATION_NOISE = 0.1;
            static constexpr TI STEP_LIMIT = 1000000;
            static constexpr TI N_WARMUP_STEPS = 20000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 256;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
//            static constexpr bool SHARED_BATCH = false;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
