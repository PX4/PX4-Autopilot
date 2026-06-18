#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_PERSIST_CODE_H
#include "../containers/matrix/persist_code.h"
#include "../persist/code.h"
#include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    std::string to_string(nn::LayerCapability capability){
        switch(capability){
            case nn::LayerCapability::Forward:
                return "Forward";
            case nn::LayerCapability::Backward:
                return "Backward";
            case nn::LayerCapability::Gradient:
                return "Gradient";
            default:
                return "UnknownCapability";
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif