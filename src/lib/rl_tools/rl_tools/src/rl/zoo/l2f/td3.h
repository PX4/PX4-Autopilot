#include "environment.h"


#include <rl_tools/rl/algorithms/td3/loop/core/config.h>
#include <rl_tools/utils/generic/typing.h>

namespace rl_tools::rl::zoo::l2f::td3{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_TINY_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct TD3_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, TI>{
                static constexpr TI ACTOR_BATCH_SIZE = 256;
                static constexpr TI CRITIC_BATCH_SIZE = 256;
                static constexpr TI TRAINING_INTERVAL = 16;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TARGET_UPDATE_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.25;
                static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.1;
                static constexpr T GAMMA = 0.99;
                static constexpr bool IGNORE_TERMINATION = false;
            };
            static constexpr T EXPLORATION_NOISE = 0.3;
            static constexpr TI STEP_LIMIT = 400000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 32;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 32;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI EPISODE_STEP_LIMIT = 500;
            static constexpr TI N_WARMUP_STEPS = 10000;
//            static constexpr bool SHARED_BATCH = false;
            struct OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 3e-4;
                static constexpr bool ENABLE_GRADIENT_CLIPPING = false;
                static constexpr T GRADIENT_CLIP_VALUE = 1;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
                static constexpr T WEIGHT_DECAY = 0.0001;
            };

        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
