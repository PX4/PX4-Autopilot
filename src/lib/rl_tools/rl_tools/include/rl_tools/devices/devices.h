#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_DEVICES_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_DEVICES_H

#include "../rl_tools.h"

#include "../utils/generic/typing.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    namespace devices {
        struct ExecutionHints{};
        template <typename DEV_SPEC>
        struct Device{
        };
        // todo: deprecate the global device id and move it to the cpu devices which sometimes need compatibility checks
        enum class DeviceId{
            Generic,
            Dummy,
            CPU,
            CPU_BLAS,
            CPU_MKL,
            CPU_OPENBLAS,
            CPU_ACCELERATE,
            CPU_TENSORBOARD,
            CUDA,
            ARM,
            ESP32,
            WASM32
        };
        enum class Type {
            math,
            random,
            logging
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace generic{
        namespace random{
            template <typename T_MATH_DEVICE>
            struct ENGINE{
                using STATE_TYPE = typename T_MATH_DEVICE::index_t;
                STATE_TYPE state;
            };
        }
    }
    namespace random{
        template <typename T_MATH_DEVICE>
        struct Generic{
            static constexpr Type TYPE = Type::random;
            using MATH_DEVICE = T_MATH_DEVICE;
            template <typename T_NOTHING = void>
            using ENGINE = devices::generic::random::ENGINE<MATH_DEVICE>;
        };
    }
    namespace math{
        struct Generic{
            static constexpr Type TYPE = Type::math;
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC>
    void malloc(devices::Device<DEV_SPEC>& device){}
    template <typename DEV_SPEC>
    void init(devices::Device<DEV_SPEC>& device){}
    template <typename DEV_SPEC, typename T>
    void count_malloc(devices::Device<DEV_SPEC>& device, T){}
    template <typename DEV_SPEC>
    void free(devices::Device<DEV_SPEC>& device){}
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif