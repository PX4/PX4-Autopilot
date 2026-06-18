#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_ACCELERATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_ACCELERATE_H

#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_DELAY_OPERATIONS_GENERIC_INCLUDE
#include "operations_cpu.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components::off_policy_runner{
    template<typename DEV_SPEC, typename SPEC, typename RNG>
    void prologue(devices::CPU_ACCELERATE<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, RNG &rng) {
        prologue((devices::CPU<DEV_SPEC>&)device, runner, rng);
    }
    template<typename DEV_SPEC, typename SPEC, typename RNG>
    void epilogue(devices::CPU_ACCELERATE<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, RNG &rng) {
        epilogue((devices::CPU<DEV_SPEC>&)device, runner, rng);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"
#endif
