#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CPU_H

#include "devices.h"
#include "../utils/generic/typing.h"

#include <cstddef>
#include <string>
#include <algorithm>
#include <ctime>
#include <limits>
#include <random>
#include <iostream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace cpu{
        template <typename T_MATH, typename T_RANDOM, typename T_LOGGING>
        struct Specification{
            using EXECUTION_HINTS = ExecutionHints;
            using MATH = T_MATH;
            using RANDOM = T_RANDOM;
            using LOGGING = T_LOGGING;
            using index_t = size_t;
        };
        struct Base{
            static constexpr DeviceId DEVICE_ID = DeviceId::CPU;
            using index_t = size_t;
            static constexpr index_t MAX_INDEX = std::numeric_limits<index_t>::max();
        };
    }
    namespace math{
        struct CPU: cpu::Base{
            static constexpr Type TYPE = Type::math;
        };
    }
    namespace random{
        struct CPU: devices::random::Generic<devices::math::CPU>, cpu::Base{
            static constexpr Type TYPE = Type::random;
            template <typename T_ENGINE = std::mt19937>
            struct ENGINE{
                using TYPE = T_ENGINE;
                TYPE engine;
            };
        };
    }
    namespace logging{
        struct CPU: cpu::Base{
            static constexpr Type TYPE = Type::logging;
        };
    }
    template <typename T_SPEC>
    struct CPU: Device<T_SPEC>, cpu::Base{
        using SPEC = T_SPEC;
        using EXECUTION_HINTS = typename SPEC::EXECUTION_HINTS;
        typename SPEC::MATH math;
        typename SPEC::RANDOM random;
        typename SPEC::LOGGING logger;
        bool initialized = false;
        index_t malloc_counter = 0;
    };

    using DefaultCPUSpecification = cpu::Specification<math::CPU, random::CPU, logging::CPU>;
    using DefaultCPU = CPU<DefaultCPUSpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace devices::cpu{
        void display_compile_options() {
            bool isOptimal = true;
            const char* ANSI_RED = "\033[31m";
            const char* ANSI_GREEN = "\033[32m";
            const char* ANSI_YELLOW = "\033[33m";
            const char* ANSI_RESET = "\033[0m";

            // Check debug mode
            bool isDebug =
            #if defined(_DEBUG) || defined(DEBUG)
                true;
            #else
                false;
            #endif
            if (isDebug) isOptimal = false;

            // Check optimization
            bool maxOpt =
            #if defined(_MSC_VER)
                #if defined(_FULL_OPTIMIZATION)
                    true;
                #else
                    false;
                #endif
            #elif defined(__GNUC__) || defined(__clang__)
                #if defined(__OPTIMIZE__)
                    true;
                #else
                    false;
                #endif
            #else
                false;
            #endif
            if (!maxOpt) isOptimal = false;

            // Check fast math
            bool fastMath =
            #if defined(__FAST_MATH__) || defined(_M_FP_FAST)
                true;
            #else
                false;
            #endif
            if (!fastMath) isOptimal = false;

            if (!isOptimal) {
                std::cerr << "Compiliation Options:" << std::endl;

                // Output debug status
                std::cerr << ((!isDebug) ? ANSI_GREEN : ANSI_RED) << "["
                          << ((!isDebug) ? "OK" : "NOT OK") << "]" << ANSI_RESET
                          << " Debug Mode: " << (isDebug ? "Yes" : "No") << std::endl;

                // Output optimization status
                std::cerr << (maxOpt ? ANSI_GREEN : ANSI_RED) << "["
                          << (maxOpt ? "OK" : "NOT OK") << "]" << ANSI_RESET
                          << " Optimization: " << (maxOpt ? "Yes" : "No") << std::endl;

                // Output fast math status
                std::cerr << (fastMath ? ANSI_GREEN : ANSI_RED) << "["
                          << (fastMath ? "OK" : "NOT OK") << "]" << ANSI_RESET
                          << " Fast Math: " << (fastMath ? "Yes" : "No") << std::endl;

                // Output optimization level
                std::cerr << "Optimization Level: "
                #if defined(_MSC_VER)
                    #if defined(_FULL_OPTIMIZATION)
                          << "Full (/Ox)"
                    #elif defined(_OPTIMIZATION_FULL)
                          << "Full (/O2)"
                    #elif defined(_OPTIMIZATION_SPEED)
                          << "Speed (/O2)"
                    #elif defined(_OPTIMIZATION_DEBUG)
                          << "Debug (/Od)"
                    #else
                          << "Unknown"
                    #endif
                #elif defined(__clang__)
                    #if defined(__OPTIMIZE__)
                        #if defined(__OPTIMIZE_SIZE__)
                              << "-Os"
                        #elif (__OPTIMIZE__ == 1)
                              << "-O1"
                        #elif (__OPTIMIZE__ == 2)
                              << "-O2"
                        #elif (__OPTIMIZE__ == 3)
                              << "-O3"
                        #else
                              << "Unknown level"
                        #endif
                    #else
                          << "-O0"
                    #endif
                #else
                          << "Unknown"
                #endif
                          << std::endl;

                std::cerr << ANSI_YELLOW << "WARNING: Non-optimal configuration detected!"
                          << ANSI_RESET << std::endl;
                std::cerr << "For maximum performance, compile with:" << std::endl;
                #if defined(_MSC_VER)
                    std::cerr << "MSVC: /O2 /fp:fast /DNDEBUG" << std::endl;
                #elif defined(__GNUC__) || defined(__clang__)
                    std::cerr << "GCC/Clang: -Ofast (or -DCMAKE_BUILD_TYPE=Release)" << std::endl;
                #else
                    std::cerr << "Unknown compiler: please consult compiler documentation" << std::endl;
                #endif
                std::cerr << std::endl;
            }
        }
    }
    template <typename DEV_SPEC>
    void init(devices::CPU<DEV_SPEC>& device){
        if(!device.initialized){
#ifndef __CLING__
            devices::cpu::display_compile_options();
#endif
            device.initialized = true;
        }
    }
    template <typename DEV_SPEC, typename TI>
    void count_malloc(devices::CPU<DEV_SPEC>& device, TI size){
        device.malloc_counter += size;
        if (size > 100000000){
            std::cerr << "Large malloc: " << size / 1000000 << "MB, total: " << device.malloc_counter / 1000000 << " MB" << std::endl;
        }
    }
    template <typename SPEC>
    void check_status(devices::CPU<SPEC>& device){ }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
