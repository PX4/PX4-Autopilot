/*Add commentMore actions
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>


class SCH16T_FIFO : public device::SPI, public I2CSPIDriver<SCH16T_FIFO>, public ModuleParams
{
public:
	SCH16T_FIFO(const I2CSPIDriverConfig &config);
	~SCH16T_FIFO() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:

static constexpr float FIFO_SAMPLE_DT{1e6f / 1475.f};
static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};             // 8000 Hz gyro
struct SensorStatus {
        uint16_t summary;
        uint16_t saturation;
        uint16_t common;
        uint16_t rate_common;
        uint16_t rate_x;
        uint16_t rate_y;
        uint16_t rate_z;
        uint16_t acc_x;
        uint16_t acc_y;
        uint16_t acc_z;
    };

struct SensorData {
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t temp;
};

struct RegisterConfig {
		RegisterConfig(uint16_t a = 0, uint16_t v = 0)
			: addr(a)
			, value(v)
		{};
		uint8_t addr;
		uint16_t value;
    };
    RegisterConfig _registers[6];
    SensorStatus _sensor_status;


    PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
    const spi_drdy_gpio_t _drdy_gpio;

	void ResetSpi6(bool reset);
	int probe() override;
    void exit_and_cleanup() override;
    void fpga_init(void);
    void sensor_ctrl(uint8_t fifo_rst);
    void fpga_read_config(void);
    void fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset) ;
    bool collect_and_publish();
    bool read_data() ;
    void reset_chip();
    bool read_product_id();
    void configure_registers();
    bool validate_sensor_status();
    bool validate_register_configuration();
    void read_status_registers();
    uint8_t register_read(uint16_t addr, uint32_t* value);
    void register_write(uint16_t addr, uint32_t value);
    uint64_t transfer_spi_frame(uint64_t frame);
    uint8_t calculate_crc8(uint64_t frame);
    void fpga_read_fifo8(uint16_t reg, uint8_t* value, uint8_t size) ;
    void fpga_write_reg8(uint16_t reg, uint8_t value) ;
    uint8_t fpga_read_reg8(uint16_t reg) ;
    void fpga_write_reg16(uint16_t reg, uint16_t value) ;
    void fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size);
    void fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size);
    uint8_t wait_direct_busy(void);
    uint16_t fpga_read_reg16(uint16_t reg);
    uint16_t _sch16t_read_status(uint16_t addr);
    uint8_t fpga_test(void);

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC8 bad"))};
    perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _drdy_missed_perf{nullptr};
	perf_counter_t _perf_general_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": general error"))};
	perf_counter_t _perf_command_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": command error"))};
	perf_counter_t _perf_saturation_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": saturation error"))};
	perf_counter_t _perf_doing_initialization{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": re-initializing"))};

	uint16_t _fifo_empty_interval_us{678}; // default 678 us
    int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

    enum class State : uint8_t {
        PowerOn,
        Reset,
        Configure,
        LockConfiguration,
        Validate,
        Read,
    } _state = State::PowerOn;

    int failure_count {};
    enum Rotation rotation {};
    bool _hardware_reset_available{false};
    uint16_t backend_rate_hz;
    uint8_t accel_instance {};
    uint8_t gyro_instance = {};
    float accel_scale {};
    float gyro_scale {};
};
