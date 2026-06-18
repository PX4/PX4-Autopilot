#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NUMERIC_TYPES_CATEGORIES_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NUMERIC_TYPES_CATEGORIES_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::numeric_types::categories {
    struct Parameter{};
    struct Accumulator{};
    struct Gradient{};
    struct OptimizerState{};
    struct Activation{};
    struct Buffer{};
    struct Input{};
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif