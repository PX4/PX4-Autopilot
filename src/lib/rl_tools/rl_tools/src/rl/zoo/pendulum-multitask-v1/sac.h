#include "environment.h"

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::pendulum_multitask_v1::sac{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            using T = typename TYPE_POLICY::DEFAULT;
            struct SAC_PARAMETERS: rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
                static constexpr TI ACTOR_BATCH_SIZE = 100;
                static constexpr TI CRITIC_BATCH_SIZE = 100;
                static constexpr TI SEQUENCE_LENGTH = 40;
            };
            static constexpr TI STEP_LIMIT = 100000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI N_WARMUP_STEPS = 1000;
            static constexpr TI N_WARMUP_STEPS_CRITIC = 1000;
            static constexpr TI N_WARMUP_STEPS_ACTOR = 1000;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 32;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 64;
            static constexpr T ALPHA = 1.0;
            static constexpr TI N_ENVIRONMENTS = 32;
            struct BATCH_SAMPLING_PARAMETERS{
                static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
                static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
                static constexpr bool RANDOM_SEQ_LENGTH = true;
                static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
                static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.3;
            };
            struct OPTIMIZER_PARAMETERS_COMMON: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
                static constexpr T GRADIENT_CLIP_VALUE = 0.001;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
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
        };
        template <typename BASE>
        struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{
            static constexpr bool DETERMINISTIC_INITIAL_STATE = false;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsGRU, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
