#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_LOGGING_OPERATIONS_ARM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_LOGGING_OPERATIONS_ARM_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE>
    void init(DEVICE& device, devices::logging::ARM& logger){ }
    template <typename DEVICE, typename A>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a){ }
    template <typename DEVICE, typename A, typename B>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b){}
    template <typename DEVICE, typename A, typename B, typename C>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c){}
    template <typename DEVICE, typename A, typename B, typename C, typename D>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e, const F f){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i){ }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I, typename J>
    void log(DEVICE& device, devices::logging::ARM& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i, const J j){ }
    template <typename DEVICE>
    void set_step(DEVICE& device, devices::logging::ARM& logger, typename DEVICE::index_t step){ /* noop */ }
    template <typename DEVICE, typename ARG_1, typename ARG_2>
    void construct(DEVICE& device, devices::logging::ARM& logger, ARG_1, ARG_2){ /* noop */ }
    template <typename DEVICE>
    void construct(DEVICE& device, devices::logging::ARM& logger){ /* noop */ }
    template <typename DEVICE>
    void free(DEVICE& device, devices::logging::ARM& logger){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG>
    void add_scalar(DEVICE& device, devices::logging::ARM& logger, const TOPIC, const ARG){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename CADENCE>
    void add_scalar(DEVICE& device, devices::logging::ARM& logger, const TOPIC, const ARG, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN, typename CADENCE>
    void add_histogram(DEVICE& device, devices::logging::ARM& logger, const TOPIC, const ARG*, const ARG_LEN, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN>
    void add_histogram(DEVICE& device, devices::logging::ARM& logger, const TOPIC, const ARG*, const ARG_LEN){ /* noop */ }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
