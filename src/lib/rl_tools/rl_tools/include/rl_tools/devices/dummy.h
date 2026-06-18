#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_DUMMY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_DUMMY_H

#include "../utils/generic/typing.h"
#include "devices.h"
#include <cstdint>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace dummy{
        struct Base{
            static constexpr DeviceId DEVICE_ID = DeviceId::Dummy;
            using index_t = unsigned long long;
            static constexpr index_t MAX_INDEX = -1;
        };
    }
    namespace math{
        struct Dummy: dummy::Base{
            static constexpr Type TYPE = Type::math;
        };
    }
    namespace random{
        struct Dummy:devices::random::Generic<devices::math::Dummy>, dummy::Base{
            static constexpr Type TYPE = Type::random;
            template <typename T_ENGINE = void>
            struct ENGINE{
                unsigned state;
            };
        };
    }
    namespace logging{
        struct Dummy: dummy::Base{
            static constexpr Type TYPE = Type::logging;
        };
    }
    template <typename T_SPEC>
    struct Dummy: Device<T_SPEC>, dummy::Base{
        template <typename OTHER_DEVICE>
        static constexpr bool compatible = utils::typing::is_same_v<OTHER_DEVICE, Dummy<T_SPEC>>;
        using SPEC = T_SPEC;
        typename SPEC::LOGGING logger;
        typename SPEC::MATH math;
        typename SPEC::RANDOM random;
    };
    struct DefaultDummySpecification{
        using MATH = math::Dummy;
        using RANDOM = random::Dummy;
        using LOGGING = logging::Dummy;
    };
    using DefaultDummy = Dummy<DefaultDummySpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
