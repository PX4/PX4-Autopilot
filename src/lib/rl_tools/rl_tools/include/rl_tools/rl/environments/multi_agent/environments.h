#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_ENVIRONMENTS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_ENVIRONMENTS_H

#include "../environments.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::multi_agent{
    template <typename T, typename TI>
    struct Environment: rl_tools::rl::environments::Environment<T, TI>{

    };
    struct DummyUI{};
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
