#include "environment.h"


#include <rl_tools/rl/algorithms/sac/loop/core/config.h>


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::acrobot_swingup_v0::sac{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, typename TYPE_POLICY::DEFAULT, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
                static constexpr TI ACTOR_BATCH_SIZE = 256;
                static constexpr TI CRITIC_BATCH_SIZE = 256;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 2;
                static constexpr T GAMMA = 0.9975;
            };
            static constexpr TI STEP_LIMIT = 1000000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 256;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr T ALPHA = 1;
            static constexpr T TARGET_ENTROPY = -2;
            static constexpr TI EPISODE_STEP_LIMIT = 20 / ENVIRONMENT::Parameters::DT;
            static constexpr TI N_WARMUP_STEPS = 50000;
            struct INITIALIZER_SPEC: nn::layers::dense::KaimingUniformSpecification<TYPE_POLICY, TI>{
                static constexpr bool INIT_LEGACY = false;
                static constexpr T SCALE = 1;
            };
            using INITIALIZER = nn::layers::dense::KaimingUniform<INITIALIZER_SPEC>;
            struct OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 1e-3;
                static constexpr bool ENABLE_BIAS_LR_FACTOR = true;
                static constexpr T BIAS_LR_FACTOR = 10;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
                static constexpr T WEIGHT_DECAY = 0;
                static constexpr T WEIGHT_DECAY_INPUT = 0;
                static constexpr T WEIGHT_DECAY_OUTPUT = 0;
            };
        };
        template <typename BASE>
        struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{
//            static constexpr TI EPISODE_STEP_LIMIT = 20 / ENVIRONMENT_PARAMETERS::DT;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
