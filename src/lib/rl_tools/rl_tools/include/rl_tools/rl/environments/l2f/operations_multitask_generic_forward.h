#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_MULTITASK_GENERIC_FORWARD_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_MULTITASK_GENERIC_FORWARD_H

#include "multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif
// forward declarations for the multi-task quadrotor such that the observe funcations can be called by the generic operations of the default quadrotor
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename RNG, bool T_OVERWRITE=false>
    static void sample_initial_parameters(DEVICE& device, rl::environments::MultirotorMultiTask<SPEC>& env, typename rl::environments::MultirotorMultiTask<SPEC>::Parameters& parameters, RNG& rng);
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
