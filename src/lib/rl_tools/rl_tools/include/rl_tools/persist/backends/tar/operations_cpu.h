#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_CPU)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_CPU

#include "io.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE>
    void write(DEVICE& device, persist::backends::tar::Writer& writer, const char* data, typename DEVICE::index_t size) {
        using TI = typename DEVICE::index_t;
        for (TI i = 0; i < size; i++) {
            writer.buffer.push_back(data[i]);
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#ifndef RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_CPU_NOT_INCLUDE_GENERIC
#include "operations_generic.h"
#endif
#endif
