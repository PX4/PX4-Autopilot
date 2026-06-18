#include "environment.h"

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::flag::sac{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        static constexpr bool SEQUENCE_MODELS = true;
        static constexpr TI MAX_EPISODE_LENGTH = 200;
        static constexpr bool PRIVILEGED_OBSERVATION = false;
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI, MAX_EPISODE_LENGTH, PRIVILEGED_OBSERVATION>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
                static constexpr TI ACTOR_BATCH_SIZE = 32;
                static constexpr TI CRITIC_BATCH_SIZE = 32;
                static constexpr T GAMMA = 0.9908096983715691; //math::pow(typename DEVICE::SPEC::MATH{}, 0.95, 36.0 / MAX_EPISODE_LENGTH);
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 2;
                static constexpr TI SEQUENCE_LENGTH = SEQUENCE_MODELS ? ENVIRONMENT::EPISODE_STEP_LIMIT : 1;
                static constexpr bool ENTROPY_BONUS = true;
                static constexpr bool ENTROPY_BONUS_NEXT_STEP = false;
                static constexpr T TARGET_ENTROPY = -2;
                static constexpr T ALPHA = 1;
                static constexpr bool ADAPTIVE_ALPHA = true;
            };
            static constexpr TI STEP_LIMIT = 400000;
            static constexpr TI N_ENVIRONMENTS = 128;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI N_WARMUP_STEPS = 1000;
            static constexpr TI N_WARMUP_STEPS_CRITIC = 1000;
            static constexpr TI N_WARMUP_STEPS_ACTOR = 1000;
            static constexpr TI ACTOR_NUM_LAYERS = 4;
            static constexpr TI ACTOR_HIDDEN_DIM = 48;
            static constexpr TI CRITIC_NUM_LAYERS = 4;
            static constexpr TI CRITIC_HIDDEN_DIM = 48;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr T ALPHA = 1.0;

            // using BATCH_SAMPLING_PARAMETERS = rl::components::off_policy_runner::SequentialBatchParameters<T, TI, SAC_PARAMETERS::SEQUENCE_LENGTH>;
            struct BATCH_SAMPLING_PARAMETERS{
                static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = SEQUENCE_MODELS;
                static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = SEQUENCE_MODELS;
                static constexpr bool RANDOM_SEQ_LENGTH = true;
                static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
                static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.1;
            };

            struct ACTOR_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 1e-4;
            };
            struct CRITIC_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 1e-3;
            };
            struct ALPHA_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 1e-3;
            };
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsGRU, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
