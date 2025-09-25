#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>

static constexpr uint8_t I2C_ADDR      = 0x28;
static constexpr uint8_t REG_WHOAMI    = 0x00;
static constexpr uint8_t REG_DATA      = 0x10;
static constexpr uint8_t WHOAMI_EXPECT = 0xA5;

class MyMcuI2C : public device::I2C, public px4::ScheduledWorkItem {
public:
    MyMcuI2C(int bus, uint8_t addr = I2C_ADDR)
    : I2C("MyMcuI2C", nullptr, bus, addr, 400000),
      ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}

    int init() override {
        if (I2C::init() != PX4_OK) { PX4_ERR("i2c init fail"); return PX4_ERROR; }
        uint8_t who{};
        if (readReg(REG_WHOAMI, who) != PX4_OK || who != WHOAMI_EXPECT) {
            PX4_WARN("whoami 0x%02x", who);
        }
        ScheduleOnInterval(20_ms);
        return PX4_OK;
    }

    void Run() override {
        if (should_exit()) { ScheduleClear(); return; }
        uint8_t buf[5]{};
        if (readBlock(REG_DATA, buf, sizeof(buf)) != PX4_OK) return;
        // demo: 将前4字节当float发布到debug_key_value
        float v{}; memcpy(&v, buf, sizeof(float));
        debug_key_value_s msg{};
        msg.timestamp = hrt_absolute_time();
        strncpy(msg.key, "mcu_val", sizeof(msg.key)-1);
        msg.value = v;
        _dbg_pub.publish(msg);
    }

private:
    int readReg(uint8_t reg, uint8_t &val) {
        return transfer(&reg, 1, &val, 1);
    }
    int readBlock(uint8_t reg, void *dst, size_t n) {
        return transfer(&reg, 1, (uint8_t *)dst, n);
    }

    uORB::Publication<debug_key_value_s> _dbg_pub{ORB_ID(debug_key_value)};
};

extern "C" __EXPORT int my_mcu_i2c_main(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "start") != 0) {
        PX4_INFO("usage: my_mcu_i2c start [bus]");
        return 0;
    }
    int bus = (argc >= 3) ? atoi(argv[2]) : PX4_I2C_BUS_EXPANSION;
    static MyMcuI2C drv(bus);
    return drv.init();
}


