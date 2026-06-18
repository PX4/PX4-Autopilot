#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_ARM_GROUP_1_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_ARM_GROUP_1_H

#if defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_ARM_GROUP_1)
    #define RL_TOOLS_OPERATIONS_ARM_GROUP_1
    #include "../../devices/arm.h"
    #include "../../numeric_types/policy.h"
//    #include "../../utils/assert/declarations_arm.h"
    #include "../../math/operations_arm.h"
    #include "../../logging/operations_arm.h"
    #include "../../utils/env/operations_generic.h"
#else
    #error "Group 1 already imported"
#endif

#endif