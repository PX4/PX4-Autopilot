#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_PERSIST_H


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, rl::components::on_policy_runner::Dataset<SPEC>& dataset, GROUP& group){
        save(device, dataset.data, group, "data");
        save(device, dataset.all_observations_privileged, group, "all_observations");
        save(device, dataset.observations, group, "observations");
        save(device, dataset.actions, group, "actions");
        save(device, dataset.action_log_probs, group, "action_log_probs");
        save(device, dataset.rewards, group, "rewards");
        save(device, dataset.terminated, group, "terminated");
        save(device, dataset.truncated, group, "truncated");
        save(device, dataset.all_values, group, "all_values");
        save(device, dataset.values, group, "values");
        save(device, dataset.advantages, group, "advantages");
        save(device, dataset.target_values, group, "target_values");
    }
    template <typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, rl::components::on_policy_runner::Dataset<SPEC>& dataset, GROUP& group){
        bool success = load(device, dataset.data, group, "data");
        success &= load(device, dataset.all_observations_privileged, group, "all_observations");
        success &= load(device, dataset.observations, group, "observations");
        success &= load(device, dataset.actions, group, "actions");
        success &= load(device, dataset.action_log_probs, group, "action_log_probs");
        success &= load(device, dataset.rewards, group, "rewards");
        success &= load(device, dataset.terminated, group, "terminated");
        success &= load(device, dataset.truncated, group, "truncated");
        success &= load(device, dataset.all_values, group, "all_values");
        success &= load(device, dataset.values, group, "values");
        success &= load(device, dataset.advantages, group, "advantages");
        success &= load(device, dataset.target_values, group, "target_values");
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
