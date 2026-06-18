#include "environment_big.h"

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

#include <rl_tools/utils/generic/typing.h>

namespace rl_tools::rl::zoo::l2f::sac{
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION=true>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        struct OPTIONS{
            static constexpr bool SEQUENTIAL_MODEL = false;
            static constexpr bool MOTOR_DELAY = true;
            static constexpr bool RANDOMIZE_MOTOR_MAPPING = false;
            static constexpr bool RANDOMIZE_THRUST_CURVES = false;
            static constexpr bool OBSERVE_THRASH_MARKOV = false;
        };
        using ENVIRONMENT = typename ENVIRONMENT_BIG_FACTORY<DEVICE, TYPE_POLICY, TI, OPTIONS>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS: rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI>{
                static constexpr TI ACTOR_BATCH_SIZE = 512;
                static constexpr TI CRITIC_BATCH_SIZE = 512;
                static constexpr TI TRAINING_INTERVAL = 16;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr T GAMMA = 0.99;
                static constexpr bool IGNORE_TERMINATION = false;
                // static constexpr T ALPHA = 0.05;
                static constexpr T TARGET_ENTROPY = -((T)2);
                static constexpr TI SEQUENCE_LENGTH = 1;
                static constexpr bool ENTROPY_BONUS_NEXT_STEP = false;
            };
            static constexpr TI N_ENVIRONMENTS = 1;
            static constexpr TI STEP_LIMIT = 5000000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 32;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI EPISODE_STEP_LIMIT = 500;
        //            static constexpr bool SHARED_BATCH = false;
            static constexpr TI N_WARMUP_STEPS = 0; // Exploration executed with a uniform random policy for N_WARMUP_STEPS steps
            static constexpr TI N_WARMUP_STEPS_CRITIC = 10000; // Number of steps before critic training starts
            static constexpr TI N_WARMUP_STEPS_ACTOR = 10000; // Number of steps before actor training starts
            struct OPTIMIZER_PARAMETERS_COMMON: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr bool ENABLE_GRADIENT_CLIPPING = false;
                static constexpr T GRADIENT_CLIP_VALUE = 1;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
                static constexpr T WEIGHT_DECAY = 0.0001;
            };
            struct ACTOR_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 3e-4;
            };
            struct CRITIC_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 3e-4;
            };
            struct ALPHA_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 1e-4;
            };
            static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = true;
            struct BATCH_SAMPLING_PARAMETERS{
                static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = false;
                static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
                static constexpr bool RANDOM_SEQ_LENGTH = false;
                static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
                static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.1;
            };
        };
        // this config is competitive with mlp but 15x slower

        using LOOP_CORE_CONFIG = rl_tools::utils::typing::conditional_t<OPTIONS::SEQUENTIAL_MODEL,
            rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rl::algorithms::sac::loop::core::ConfigApproximatorsGRU, DYNAMIC_ALLOCATION>,
            rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rl::algorithms::sac::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>
        >;
    };
}
