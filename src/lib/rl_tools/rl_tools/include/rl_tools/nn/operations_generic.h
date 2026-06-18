#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_OPERATIONS_GENERIC_H
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

/*
 * Generic operations can run on the CPU or GPU depending on the setting of the RL_TOOLS_FUNCTION_PLACEMENT macro.
 */

#include "layers/operations_generic.h"
#include "loss_functions/operations_generic.h"


#endif