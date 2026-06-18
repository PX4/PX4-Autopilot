#include <rl_tools/operations/cpu_tensorboard.h>
namespace rlt = rl_tools;


#include <gtest/gtest.h>


TEST(RL_TOOLS_LOGGING_TENSORBOARD, INIT){

    using LOGGER = rlt::devices::logging::CPU_TENSORBOARD<rlt::devices::logging::CPU_TENSORBOARD_FREQUENCY_EXTENSION>;
    using DEV_SPEC = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, LOGGER>;
    using DEVICE = rlt::devices::CPU<DEV_SPEC>;
    using TI = typename DEVICE::index_t;
    using T = float;

    DEVICE device;
    rlt::construct(device, device.logger);

    for(TI i = 0; i < 100; i++){
        rlt::set_step(device, device.logger, i);
        rlt::add_scalar(device, device.logger, "test", (T)(i * i));
    }

    rlt::free(device, device.logger);

}
