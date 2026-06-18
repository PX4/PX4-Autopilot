#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_ARM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_ARM_H

#include "../utils/generic/typing.h"
#include "devices.h"

#include <cstddef>
#include <cstdint>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace arm{
        template <typename T_MATH, typename T_RANDOM, typename T_LOGGING>
        struct Specification{
            using MATH = T_MATH;
            using RANDOM = T_RANDOM;
            using LOGGING = T_LOGGING;
        };
        struct Base{
            static constexpr DeviceId DEVICE_ID = DeviceId::ARM;
            using index_t = size_t;
            static constexpr index_t MAX_INDEX = -1;

        };
    }
    namespace math{
        struct ARM: arm::Base{
            static constexpr Type TYPE = Type::math;
        };
    }
    namespace random{
        struct ARM: devices::random::Generic<typename math::ARM>, arm::Base{
            static constexpr Type TYPE = Type::random;
            template <bool T_DYNAMIC_ALLOCATION=false>
            struct EngineSpecification{
                static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            };
            template <typename SPEC = EngineSpecification<>>
            struct ENGINE{
                arm::Base::index_t state;
            };
        };
    }
    namespace logging{
        struct ARM: arm::Base{
            static constexpr Type TYPE = Type::logging;
        };
        struct ARDUINO: arm::Base{
            static constexpr Type TYPE = Type::logging;
        };
    }
    template <typename T_SPEC>
    struct ARM: Device<T_SPEC>, arm::Base{
        template <typename OTHER_DEVICE>
        static constexpr bool compatible = OTHER_DEVICE::DEVICE == DeviceId::ARM;
        using SPEC = T_SPEC;
        typename SPEC::MATH math;
        typename SPEC::RANDOM random;
        typename SPEC::LOGGING logger;
#ifdef RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
        index_t malloc_counter = 0;
#endif
    };

    namespace arm{
        template <typename T_SPEC>
        struct Generic: ARM<T_SPEC>{};
        template <typename T_SPEC>
        struct DSP: ARM<T_SPEC>{};
        template <typename T_SPEC>
        struct OPT: ARM<T_SPEC>{};
    }

    using DefaultARMSpecification = arm::Specification<math::ARM, random::ARM, logging::ARM>;
    using DefaultARM = arm::OPT<DefaultARMSpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
#ifdef RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
    template <typename DEV_SPEC, typename TI>
    void count_malloc(devices::ARM<DEV_SPEC>& device, TI size){
        device.malloc_counter += size;
    }
#endif
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
