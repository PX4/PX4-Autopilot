#include "version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_TOOLS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_TOOLS_H
//#define RL_TOOLS_NAMESPACE_WRAPPER test_ns
#ifndef RL_TOOLS_NAMESPACE_WRAPPER
#define RL_TOOLS_NAMESPACE_WRAPPER
#define RL_TOOLS_NAMESPACE_WRAPPER_START
#define RL_TOOLS_NAMESPACE_WRAPPER_END
#else
#define RL_TOOLS_NAMESPACE RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools
#define RL_TOOLS_NAMESPACE_WRAPPER_START namespace RL_TOOLS_NAMESPACE_WRAPPER {
#define RL_TOOLS_NAMESPACE_WRAPPER_END }
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <auto T_C>
    struct Constant{
        static constexpr auto C = T_C;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#if defined(_MSC_VER)
    #define RL_TOOLS_RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
    #define RL_TOOLS_RESTRICT __restrict__
#else
    #define RL_TOOLS_RESTRICT
#endif


#endif

//#define RL_TOOLS_TARGET_COMMIT_HASH 111f0b82ba87ee4f1938f4d85ee2d266e70f4498
//#define RL_TOOLS_TARGET_COMMIT_HASH_SHORT 1122059
