#include "../rl_tools.h"
#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NUMERIC_TYPES_BF16_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NUMERIC_TYPES_BF16_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace numeric_types {

#ifdef RL_TOOLS_NUMERIC_TYPES_ENABLE_BF16
        using bf16 = __bf16;
        static_assert(sizeof(bf16) == 2);
#endif
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// namespace std{
//     std::string to_string(rl_tools::numeric_types::bf16 value){
//         return std::to_string(static_cast<float>(value));
//     }
//     rl_tools::numeric_types::bf16 exp(rl_tools::numeric_types::bf16 value){
//         return static_cast<rl_tools::numeric_types::bf16>(std::exp(static_cast<float>(value)));
//     }
// }

#endif