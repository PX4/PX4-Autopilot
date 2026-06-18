#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_ESP32_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_ESP32_H


#ifndef RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    constexpr bool compile_time_redefinition_detector = true; // When importing different devices don't import the full header. The operations need to be imporeted interleaved (e.g. include cpu group 1 -> include cuda group 1 -> include cpu group 2 -> include cuda group 2 -> ...)
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

#include "esp32/group_1.h"
#include "esp32/group_2.h"
#include "esp32/group_3.h"

#endif