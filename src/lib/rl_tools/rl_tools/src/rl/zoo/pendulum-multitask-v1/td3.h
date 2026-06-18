#include "environment.h"

#include <rl_tools/rl/algorithms/td3/loop/core/config.h>


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::pendulum_multitask_v1::td3{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG, bool DYNAMIC_ALLOCATION>
    struct FACTORY{
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>::ENVIRONMENT;
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
            struct TD3_PARAMETERS: rl::algorithms::td3::DefaultParameters<TYPE_POLICY, TI>{};
            static constexpr TI STEP_LIMIT = 20000;
            static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 64;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 64;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
