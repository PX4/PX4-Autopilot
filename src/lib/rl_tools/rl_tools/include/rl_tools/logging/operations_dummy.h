#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_LOGGING_OPERATIONS_DUMMY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_LOGGING_OPERATIONS_DUMMY_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename A>
    void log(DEVICE& device, devices::logging::Dummy& logger, const A a){
    }
    template <typename DEVICE, typename A, typename B>
    void log(DEVICE& device, devices::logging::Dummy& logger, const A a, const B b){
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D>
    void log(DEVICE& device, devices::logging::Dummy& logger, const A a, const B b, const C c, const D d){
    }
    template <typename DEVICE>
    void set_step(DEVICE& device, devices::logging::Dummy& logger, typename DEVICE::index_t step){ /* noop */ }
    template <typename DEVICE, typename ARG_1, typename ARG_2>
    void construct(DEVICE& device, devices::logging::Dummy& logger, ARG_1, ARG_2){ /* noop */ }
    template <typename DEVICE>
    void construct(DEVICE& device, devices::logging::Dummy& logger){ /* noop */ }
    template <typename DEVICE>
    void free(DEVICE& device, devices::logging::Dummy& logger){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG>
    void add_scalar(DEVICE& device, devices::logging::Dummy& logger, const TOPIC, const ARG){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename CADENCE>
    void add_scalar(DEVICE& device, devices::logging::Dummy& logger, const TOPIC, const ARG, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN, typename CADENCE>
    void add_histogram(DEVICE& device, devices::logging::Dummy& logger, const TOPIC, const ARG*, const ARG_LEN, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN>
    void add_histogram(DEVICE& device, devices::logging::Dummy& logger, const TOPIC, const ARG*, const ARG_LEN){ /* noop */ }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
