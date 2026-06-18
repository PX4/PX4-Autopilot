#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_WASM32_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_WASM32_H

#include "../utils/generic/typing.h"
#include "devices.h"
#include <stdint.h>
/*
 * This devices is a "pure" WASM32 device, meaning that it is barebones enough to be run with e.g. `clang++ -Irl_tools/include -std=c++17 --target=wasm32 -nostdlib -Wl,--no-entry -Wl,--export=some_fn -o interface.wasm interface.cpp` (see ./src/nn_models/import_wasm)
 * If you want a more versatile device, that can use the advanced syscalls in e.g. Emscripten, that should be a separate device
 */
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace wasm32{
        struct Base{
            static constexpr DeviceId DEVICE_ID = DeviceId::WASM32;
            using index_t = uint32_t;
            static constexpr index_t MAX_INDEX = -1;
        };
    }
    namespace math{
        struct WASM32: wasm32::Base{
            static constexpr Type TYPE = Type::math;
        };
    }
    namespace random{
        struct WASM32:devices::random::Generic<devices::math::WASM32>, wasm32::Base{
            static constexpr Type TYPE = Type::random;
            template <typename T_ENGINE = void>
            struct ENGINE{
                unsigned int state;
            };
        };
    }
    namespace logging{
        struct WASM32: wasm32::Base{
            static constexpr Type TYPE = Type::logging;
        };
    }
    template <typename T_SPEC>
    struct WASM32: Device<T_SPEC>, wasm32::Base{
        template <typename OTHER_DEVICE>
        static constexpr bool compatible = utils::typing::is_same_v<OTHER_DEVICE, WASM32<T_SPEC>>;
        using SPEC = T_SPEC;
        typename SPEC::LOGGING logger;
        typename SPEC::MATH math;
        typename SPEC::RANDOM random;
    };
    struct DefaultWASM32Specification{
        using MATH = math::WASM32;
        using RANDOM = random::WASM32;
        using LOGGING = logging::WASM32;
    };
    using DefaultWASM32 = WASM32<DefaultWASM32Specification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
