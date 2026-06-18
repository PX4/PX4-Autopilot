#include "environment_tiny.h"

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

#include <rl_tools/utils/generic/typing.h>


// run this as MKL_NUM_THREADS=1 /usr/bin/time sudo -E nice -n-20 cmake-build-release/src/rl/zoo/rl_zoo_l2f_sac_benchmark
// in the best case this yields < 1s on an AMD Ryzen 9 7945HX
// Time: 0.802s for 18bf22433e7a6b075d864088ac4768a51f72a75b when using 7945HX


namespace rl_tools::rl::zoo::l2f::sac{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION=true>
    struct FACTORY{
        using ENVIRONMENT = typename ENVIRONMENT_TINY_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI>{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr TI ACTOR_BATCH_SIZE = 32;
                static constexpr TI CRITIC_BATCH_SIZE = 32;
                static constexpr TI TRAINING_INTERVAL = 2;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr T GAMMA = 0.98;
                static constexpr bool IGNORE_TERMINATION = false;
                static constexpr T TARGET_ENTROPY = -((T)4);
                static constexpr TI SEQUENCE_LENGTH = 1;
            };
            static constexpr TI STEP_LIMIT = 35000;
            static constexpr TI REPLAY_BUFFER_CAP = 10000;
            static constexpr TI ACTOR_NUM_LAYERS = 5;
            static constexpr TI ACTOR_HIDDEN_DIM = 16;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI CRITIC_NUM_LAYERS = 5;
            static constexpr TI CRITIC_HIDDEN_DIM = 16;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI EPISODE_STEP_LIMIT = 500;
            static constexpr TI N_WARMUP_STEPS = 200;
            static constexpr TI N_WARMUP_STEPS_CRITIC = 200;
            static constexpr TI N_WARMUP_STEPS_ACTOR = 200;
            static constexpr bool SHARED_BATCH = true;
            static constexpr bool COLLECT_EPISODE_STATS = false;
            static constexpr TI N_ENVIRONMENTS = 8;
            using T = typename TYPE_POLICY::DEFAULT;
            struct OPTIMIZER_PARAMETERS_COMMON: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr bool ENABLE_GRADIENT_CLIPPING = false;
                static constexpr T GRADIENT_CLIP_VALUE = 1;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
                static constexpr T WEIGHT_DECAY = 0.0001;
            };
            struct ACTOR_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T ALPHA = 1e-3;
            };
            struct CRITIC_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T ALPHA = 2e-3;
            };
            struct ALPHA_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T ALPHA = 1e-3;
            };
            static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = true;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
