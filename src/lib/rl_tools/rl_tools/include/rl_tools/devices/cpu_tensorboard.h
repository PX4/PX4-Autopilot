#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CPU_TENSORBOARD_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CPU_TENSORBOARD_H

#include "devices.h"
#include "cpu.h"

#include <tensorboard_logger/tensorboard_logger.h>
#include <mutex>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace logging{
        struct CPU_TENSORBOARD_FREQUENCY_EXTENSION: logging::CPU{
            using TI = typename logging::CPU::index_t;
            std::map<std::string, TI> topic_frequency_dict;
        };
        template <typename PARENT=logging::CPU>
        struct CPU_TENSORBOARD: PARENT{
            using TI = typename logging::CPU::index_t;
            static constexpr DeviceId DEVICE_ID = DeviceId::CPU_TENSORBOARD;
            static constexpr Type TYPE = Type::logging;
            TI step = 0;
            TensorBoardLogger* tb = nullptr;
            std::mutex mutex;
        };
    }
    using DefaultCPU_TENSORBOARDSpecification = cpu::Specification<math::CPU, random::CPU, logging::CPU_TENSORBOARD<>>;
    using DefaultCPU_TENSORBOARD = CPU<DefaultCPU_TENSORBOARDSpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif