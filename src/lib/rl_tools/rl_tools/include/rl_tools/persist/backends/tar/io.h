#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_TAR_IO_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_TAR_IO_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::persist::backends::tar {
    struct Writer{
        std::vector<char> buffer;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




