#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_LOGGING_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_LOGGING_OPERATIONS_CPU_H



#include <iostream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE>
    void init(DEVICE& dev, devices::logging::CPU& logger){ }
    template <typename DEVICE, typename A>
    void log(DEVICE& dev, devices::logging::CPU& logger, const A a){
        std::cout << a << std::endl;
    }
    template <typename DEVICE, typename A, typename B>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b){
        std::cout << a << b << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c){
        std::cout << a << b << c << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d){
        std::cout << a << b << c << d << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e){
        std::cout << a << b << c << d << e << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f){
        std::cout << a << b << c << d << e << f << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g){
        std::cout << a << b << c << d << e << f << g << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h){
        std::cout << a << b << c << d << e << f << g << h << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i){
        std::cout << a << b << c << d << e << f << g << h << i << std::endl;
    }
    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I, typename J>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i, const J j){
        std::cout << a << b << c << d << e << f << g << h << i << j << std::endl;
    }

    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I, typename J, typename K>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i, const J j, const K k){
        std::cout << a << b << c << d << e << f << g << h << i << j << k << std::endl;
    }

    template <typename DEVICE, typename A, typename B, typename C, typename D, typename E, typename F, typename G, typename H, typename I, typename J, typename K, typename L>
    void log(DEVICE& device, devices::logging::CPU& logger, const A a, const B b, const C c, const D d, const E e, const F f, const G g, const H h, const I i, const J j, const K k, const L l){
        std::cout << a << b << c << d << e << f << g << h << i << j << k << l << std::endl;
    }

    template <typename DEVICE>
    void set_step(DEVICE& device, devices::logging::CPU& logger, typename DEVICE::index_t step){ /* noop */ }
    template <typename DEVICE>
    typename DEVICE::index_t get_step(DEVICE& device, devices::logging::CPU& logger){return 0;}
    template <typename DEVICE, typename ARG_1, typename ARG_2>
    void construct(DEVICE& device, devices::logging::CPU& logger, ARG_1, ARG_2){ /* noop */ }
    template <typename DEVICE>
    void construct(DEVICE& device, devices::logging::CPU& logger){ /* noop */ }
    template <typename DEVICE>
    void free(DEVICE& device, devices::logging::CPU& logger){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG>
    void add_scalar(DEVICE& device, devices::logging::CPU& logger, const TOPIC, const ARG){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename CADENCE>
    void add_scalar(DEVICE& device, devices::logging::CPU& logger, const TOPIC, const ARG, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN, typename CADENCE>
    void add_histogram(DEVICE& device, devices::logging::CPU& logger, const TOPIC, const ARG*, const ARG_LEN, const CADENCE){ /* noop */ }
    template <typename DEVICE, typename TOPIC, typename ARG, typename ARG_LEN>
    void add_histogram(DEVICE& device, devices::logging::CPU& logger, const TOPIC, const ARG*, const ARG_LEN){ /* noop */ }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
