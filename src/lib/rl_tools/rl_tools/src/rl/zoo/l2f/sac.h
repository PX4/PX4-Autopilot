#include "environment.h"

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

#include <rl_tools/utils/generic/typing.h>

namespace rl_tools::rl::zoo::l2f::sac{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION=true>
    struct FACTORY{
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            using T = typename TYPE_POLICY::DEFAULT;
            struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI>{
                static constexpr TI ACTOR_BATCH_SIZE = 64;
                static constexpr TI CRITIC_BATCH_SIZE = 64;
                static constexpr TI TRAINING_INTERVAL = 5;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.9;
                static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.3;
                static constexpr T GAMMA = 0.99;
                static constexpr bool IGNORE_TERMINATION = false;
                static constexpr T TARGET_ENTROPY = -((T)4);
                static constexpr TI SEQUENCE_LENGTH = 1;
            };
            static constexpr TI STEP_LIMIT = 200000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 32;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 32;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI EPISODE_STEP_LIMIT = 500;
        //            static constexpr bool SHARED_BATCH = false;
            struct OPTIMIZER_PARAMETERS_COMMON: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
                static constexpr T GRADIENT_CLIP_VALUE = 1;
                static constexpr bool ENABLE_WEIGHT_DECAY = true;
                static constexpr T WEIGHT_DECAY = 0.0001;
            };
            struct ACTOR_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 1e-3;
            };
            struct CRITIC_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 1e-3;
            };
            struct ALPHA_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 1e-3;
            };
            static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = true;
        };

        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
