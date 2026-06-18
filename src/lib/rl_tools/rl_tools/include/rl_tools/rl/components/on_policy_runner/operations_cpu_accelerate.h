#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_OPERATIONS_CPU_ACCELERATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_OPERATIONS_CPU_ACCELERATE_H

#include "on_policy_runner.h"
#define RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_OPERATIONS_CPU_DELAY_OPERATIONS_GENERIC_INCLUDE
#include "operations_cpu.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components::on_policy_runner{
    template <typename DEV_SPEC, typename DATASET_SPEC, typename RNG> // todo: make this not PPO but general policy with output distribution
    void prologue(devices::CPU_ACCELERATE<DEV_SPEC>& device, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, rl::components::OnPolicyRunner<typename DATASET_SPEC::SPEC>& runner, RNG& rng, typename devices::CPU<DEV_SPEC>::index_t step_i){
        prologue((devices::CPU<DEV_SPEC>&)device, dataset, runner, rng, step_i);
    }
    template <typename DEV_SPEC, typename DATASET_SPEC, typename ACTIONS_MEAN_SPEC, typename ACTIONS_SPEC, typename ACTION_LOG_STD_SPEC, typename RNG> // todo: make this not PPO but general policy with output distribution
    void epilogue(devices::CPU_ACCELERATE<DEV_SPEC>& device, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, rl::components::OnPolicyRunner<typename DATASET_SPEC::SPEC>& runner, Matrix<ACTIONS_MEAN_SPEC>& actions_mean, Matrix<ACTIONS_SPEC>& actions, Matrix<ACTION_LOG_STD_SPEC>& action_log_std, RNG& rng, typename devices::CPU<DEV_SPEC>::index_t step_i){
        epilogue((devices::CPU<DEV_SPEC>&)device, dataset, runner, actions_mean, actions, action_log_std, rng, step_i);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"

#endif
