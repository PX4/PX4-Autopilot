#include "environment.h"

#include <rl_tools/rl/algorithms/td3/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::pendulum_velocity_v1::td3{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            using T = typename TYPE_POLICY::DEFAULT;
            struct TD3_PARAMETERS: rl::algorithms::td3::DefaultParameters<TYPE_POLICY, TI>{
                static constexpr TI ACTOR_BATCH_SIZE = 100;
                static constexpr TI CRITIC_BATCH_SIZE = 100;
                static constexpr TI SEQUENCE_LENGTH = 30;
                // static constexpr TI CRITIC_TRAINING_INTERVAL = ENVIRONMENT::EPISODE_STEP_LIMIT;
                // static constexpr TI ACTOR_TRAINING_INTERVAL = ENVIRONMENT::EPISODE_STEP_LIMIT;
            };
            static constexpr TI STEP_LIMIT = 200000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            // static constexpr TI N_WARMUP_STEPS = 20000;
            // static constexpr TI N_WARMUP_STEPS_CRITIC = N_WARMUP_STEPS;
            // static constexpr TI N_WARMUP_STEPS_ACTOR = N_WARMUP_STEPS;
            static constexpr TI ACTOR_NUM_LAYERS = 4;
            static constexpr TI ACTOR_HIDDEN_DIM = 16;
            static constexpr TI CRITIC_NUM_LAYERS = 4;
            static constexpr TI CRITIC_HIDDEN_DIM = 32;
            static constexpr T ALPHA = 1.0;
            static constexpr TI N_ENVIRONMENTS = 32;
            struct BATCH_SAMPLING_PARAMETERS{
                static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
                static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
                static constexpr bool RANDOM_SEQ_LENGTH = true;
                static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = false;
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
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsGRU, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
