#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_CAPABILITY_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_CAPABILITY_PERSIST_CODE_H
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <bool DYNAMIC_ALLOCATION, bool CONST>
    std::string to_string(nn::capability::Forward<DYNAMIC_ALLOCATION, CONST>){
        return std::string("RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::capability::Forward<") + (DYNAMIC_ALLOCATION ? "true" : "false") + ", " + (CONST ? "true" : "false") + ">";
    }
    template <bool DYNAMIC_ALLOCATION, bool CONST>
    std::string to_string(nn::capability::Backward<DYNAMIC_ALLOCATION, CONST>){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::capability::Backward<" + std::string((DYNAMIC_ALLOCATION ? "true" : "false")) + ", " + (CONST ? "true" : "false") + ">";
    }
    template <typename T_PARAMETER_TYPE, bool DYNAMIC_ALLOCATION, bool CONST>
    std::string to_string(nn::capability::Gradient<T_PARAMETER_TYPE, DYNAMIC_ALLOCATION, CONST>){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::capability::Gradient<"+ get_type_string(T_PARAMETER_TYPE{}) + std::string(", ") + std::string((DYNAMIC_ALLOCATION ? "true" : "false")) + ", " + (CONST ? "true" : "false") + ">";
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

