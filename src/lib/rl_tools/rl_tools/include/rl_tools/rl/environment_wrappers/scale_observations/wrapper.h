#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENT_WRAPPERS_SCALE_OBSERVATIONS_WRAPPER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENT_WRAPPERS_SCALE_OBSERVATIONS_WRAPPER_H

#include "../wrappers.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environment_wrappers::scale_observations {
    template <typename T_TYPE_POLICY, typename T_TI>
    struct Specification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr typename TYPE_POLICY::DEFAULT SCALE = 1;
    };
}
namespace rl_tools::rl::environment_wrappers{
    template <typename T_SPEC, typename T_ENVIRONMENT>
    struct ScaleObservations: Wrapper<T_ENVIRONMENT>{
        using SPEC = T_SPEC;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

