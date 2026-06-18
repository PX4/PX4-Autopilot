#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_RUNNING_NORMALIZER_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_RUNNING_NORMALIZER_PERSIST_H

#include "running_normalizer.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, rl::components::RunningNormalizer<SPEC> normalizer, GROUP& group) {
        save(device, normalizer.mean, group, "mean");
        save(device, normalizer.std, group, "std");
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
