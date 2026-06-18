#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_EXTRACK_CONFIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_EXTRACK_CONFIG_H

#include "state.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::extrack{
    template<typename T_NEXT>
    struct Config: T_NEXT {
        using NEXT = T_NEXT;
        template <typename CONFIG>
        using State = State<CONFIG>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




