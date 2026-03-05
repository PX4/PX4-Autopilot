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

#include "SCH16T_FIFO.hpp"

using namespace time_literals;

static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;  // 2 MHz SPI serial interface
static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function
static constexpr uint16_t FILTER_235HZ = (0b0000101101101);       // 235 Hz  filter
static constexpr uint16_t FILTER_68HZ = (0x0000);       // 68 Hz default filter
static constexpr uint16_t FILTER_30HZ = (0b0000001001001);       // 30 Hz  filter
static constexpr uint16_t FILTER_BYPASS = (0b0000000111111111);     // No filtering
static constexpr uint16_t RATE_300DPS_1518HZ = 0b0001001011011011; // Gyro XYZ range 300 deg/s @ 1518Hz
static constexpr uint16_t RATE_300DPS_3030HZ = 0b0001001010010010; // Gyro XYZ range 300 deg/s @ 3030Hz
static constexpr uint16_t RATE_300DPS_4419HZ = 0b0001001001001001; // Gyro XYZ range 300 deg/s @ 4419Hz
static constexpr uint16_t ACC12_8G_1518HZ = 0b0001001011011011;  // Acc XYZ range 8 G and 1518 update rate
static constexpr uint16_t ACC12_8G_3030HZ = 0b0001001010010010;     // Acc XYZ range 8 G and 3030 update rate
static constexpr uint16_t ACC12_8G_4419HZ = 0b0001001001001001;     // Acc XYZ range 8 G and 4419 update rate
static constexpr uint16_t ACC3_26G = (0b000 << 0);
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);

static constexpr uint32_t POWER_ON_TIME = 250_ms;
// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

#define T_STALL_US   20U

#define SPI32BITCONVERT2_20BIT(x) (((int32_t)(((x)<<12)& 0xfffff000UL))>>12)

#define CtrlMode_Direct 0
#define CtrlMode_FpgaRead 1

#define CTRL_Shift_Mode (0)
#define CTRL_Shift_FifoRst (1)
#define CTRL_Shift_FifoEnable (2)
#define CTRL_Shift_CmdNumSub1 (8)
#define CTRL_Shift_BaudSub1 (16)

/*
Addr_R32_Verison
check the version of FPGA
*/
#define Addr_R32_Verison (0x0000)
/*
Addr_RW16_TestReg
Test register, used for read-write testing to determine whether the FPGA is normal
*/
#define Addr_RW16_TestReg (0x0001)
/*
Addr_W32_Sch16tCtrl
bit[0]: '0' - Direct Mode, '1' - FPGA Read-Write Mode
bit[1]: '1' - Reset FIFO
bit[2]: '1' - Enable FPGA to Read Sensor data
bit[15:8]: Number of FPGA Sensor Reading Instructions - 1
bit[23:16]: SPI Clock Cycle + 1, Unit: 16.667ns. Example: 7 - Clock Cycle = (7 + 1) × 16.667 ≈ 133ns
*/
#define Addr_W32_Sch16tCtrl (0x0010)
/*
Addr_RW8_DirectRam8
Used for SPI data reading and writing in direct mode
*/
#define Addr_RW8_DirectRam8 (0x0011)
/*
Addr_RW16_DirectSpiCtrl
Write Function:
bit[3:0]: Function bits. 1 - Reset, 2 - Start Transmission
bit[15:8]: Number of transmitted bytes minus 1
Read Function:
bit[0]: SPI direct transmission busy signal. 1 - Busy, 0 - Idle
*/
#define Addr_RW16_DirectSpiCtrl (0x0012)
/*
Addr_W8_SensorReadCmdRam8
Sensor Reading Instruction RAM
Each instruction is 64 bits, occupying 8 bytes, including a 48-bit instruction and a 16-bit instruction definition.
*/
#define Addr_W8_SensorReadCmdRam8 (0x0013)
/*
Addr_W8_SensorValueFifo8
Sensor data stroge in fifo
*/
#define Addr_W8_SensorValueFifo8 (0x0014)
/*
Addr_RW8_SensorFifoCtrl
Read:
bit[7:0]: Number of packets contained in the FIFO
bit[8]: RDY signal abnormal flag
Write:
bit[0]: After reading one packet, write '1' to notify the FPGA to load the next data packet
*/
#define Addr_RW8_SensorFifoCtrl (0x0015)

#define FuncBit_Write (0 << 7)
#define FuncBit_Read (1 << 7)
#define FuncBit_Reg (0 << 5)
#define FuncBit_Ram (1 << 5)
#define FuncBit_Fifo (2 << 5)
#define FuncBit_Bit8 (0 << 3)
#define FuncBit_Bit16 (1 << 3)
#define FuncBit_Bit32 (2 << 3)


static uint8_t direct_mode = CtrlMode_Direct;
static uint8_t fifo_enable = 0;
static uint8_t fifo_cmd_num = 17;
static uint8_t fifo_baudrate = 32;

static hrt_abstime now = 0;

SCH16T_FIFO::SCH16T_FIFO(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_drdy_gpio(config.drdy_gpio)
{
	_px4_accel.set_range(163.4f); 		// 163.4 m/s2
	_px4_gyro.set_range(math::radians(5000.f));         // 5000 °/sec

	_registers[0] = RegisterConfig(CTRL_FILT_RATE,  FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
	_registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
	_registers[2] = RegisterConfig(CTRL_FILT_ACC3,  FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
	_registers[3] = RegisterConfig(CTRL_RATE,
				       RATE_300DPS_1518HZ); // +/- 300 deg/s, 1600 LSB/(deg/s) -- default, Decimation 8, 1475Hz RATE_300DPS_1475HZ
	_registers[4] = RegisterConfig(CTRL_ACC12,
				       ACC12_8G_1518HZ);   // +/- 80 m/s^2, 3200 LSB/(m/s^2) -- default, Decimation 8, 1475Hz ACC12_8G_1475HZ
	_registers[5] = RegisterConfig(CTRL_ACC3, ACC3_26G);          // +/- 260 m/s^2, 1600 LSB/(m/s^2) -- default ACC3_26G
}


SCH16T_FIFO::~SCH16T_FIFO()
{
	perf_free(_reset_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_drdy_missed_perf);
	perf_free(_perf_general_error);
	perf_free(_perf_command_error);
	perf_free(_perf_saturation_error);
	perf_free(_perf_doing_initialization);
}

int SCH16T_FIFO::init()
{
	px4_usleep(250_ms);

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	ScheduleClear();

	_state = State::PowerOn;
	ScheduleNow();

	return PX4_OK;
}

int SCH16T_FIFO::probe()
{
	return PX4_OK;
}

void SCH16T_FIFO::ResetSpi6(bool reset)
{
#if defined(SPI6_RESET)
	SPI6_RESET(reset);
#endif
}

void SCH16T_FIFO::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void SCH16T_FIFO::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_drdy_missed_perf);
	perf_print_counter(_perf_general_error);
	perf_print_counter(_perf_command_error);
	perf_print_counter(_perf_saturation_error);
	perf_print_counter(_perf_doing_initialization);
}

void SCH16T_FIFO::RunImpl()
{
	now = hrt_absolute_time();

	switch (_state) {
	case State::PowerOn: {
			_state = State::Reset;
			ScheduleDelayed(250000);
			break;
		}

	case State::Reset: {
			failure_count = 0;
			reset_chip();
			_state = State::Configure;
			ScheduleDelayed(400000);
			break;
		}

	case State::Configure: {
			if (!read_product_id()) {
				_state = State::Reset;
				ScheduleDelayed(2000000);
				break;
			}

			configure_registers();
			_state = State::LockConfiguration;
			ScheduleDelayed(250_ms);
			break;
		}

	case State::LockConfiguration: {
			read_status_registers(); // Read all status registers once
			register_write(CTRL_MODE, EOI | EN_SENSOR);// Write EOI and EN_SENSOR
			_state = State::Validate;
			ScheduleDelayed(50_ms);
			break;
		}

	case State::Validate: {
			read_status_registers(); // Read all status registers twice
			read_status_registers();

			// Check that registers are configured properly and that the sensor status is OK
			if (validate_sensor_status() && validate_register_configuration()) {
				_state = State::Read;
				fpga_init();
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

			} else {
				_state = State::Reset;
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::Read: {
			if (collect_and_publish()) {
				if (failure_count > 0) {
					failure_count--;
				}

			} else {
				failure_count++;
			}

			// Reset if successive failures
			if (failure_count > 10) {
				_state = State::Reset;
				return;
			}

			break;
		}

	default:
		break;
	} // end switch/case
}


void SCH16T_FIFO::sensor_ctrl(uint8_t fifo_rst)
{
	uint32_t reg;
	uint8_t wbuf[8];
	uint8_t ptr = 0;

	reg = (direct_mode & 1) << CTRL_Shift_Mode;
	reg |= (fifo_rst & 1) << CTRL_Shift_FifoRst;
	reg |= (fifo_enable & 1) << CTRL_Shift_FifoEnable;
	reg |= ((fifo_cmd_num - 1) & 0xff) << CTRL_Shift_CmdNumSub1;
	reg |= ((fifo_baudrate - 1) & 0xff) << CTRL_Shift_BaudSub1;

	wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit32;
	wbuf[ptr++] = Addr_W32_Sch16tCtrl & 0xff;
	wbuf[ptr++] = (Addr_W32_Sch16tCtrl >> 8) & 0xff;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = (reg >> 16) & 0xff;
	wbuf[ptr++] = (reg >> 24) & 0xff;

	transfer(wbuf, wbuf, ptr);
}

void SCH16T_FIFO::fpga_init(void)
{
	/**< fpga read */
	fifo_enable = 0;
	sensor_ctrl(1);

	fpga_read_config();

	fifo_enable = 1;
	direct_mode = CtrlMode_FpgaRead;
	sensor_ctrl(1);
}

static uint8_t gen_crc8(uint8_t *data)
{
	uint16_t crc = 0xff;
	uint8_t byte_value;

	for (uint8_t c = 0; c < 6; c++) {
		byte_value = (c == 5) ? 0x00 : data[c];

		for (uint8_t i = 0; i < 8; i++) {
			uint8_t data_bit = (byte_value >> (7 - i)) & 1;
			crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
		}
	}

	return crc;
}

static void _register_write(uint16_t addr, uint32_t value, uint8_t *buff)
{
	buff[0] = (addr >> 2) & 0xff;
	buff[1] = ((addr & 3) << 6);
	buff[1] |= 1 << 5;
	buff[1] |= 1 << 3;
	buff[2] = (value >> 16) & 0xf;
	buff[3] = (value >> 8) & 0xff;
	buff[4] = value & 0xff;
	buff[5] = gen_crc8(buff);
}

static void _register_read(uint16_t addr, uint8_t *buff)
{
	buff[0] = (addr >> 2) & 0xff;
	buff[1] = ((addr & 3) << 6);
	buff[1] |= 1 << 3;
	buff[2] = 0;
	buff[3] = 0;
	buff[4] = 0;
	buff[5] = gen_crc8(buff);
}



void SCH16T_FIFO::fpga_read_config(void)
{
	uint8_t ptr = 0;
	/**< reset fifo */
	fpga_write_cmd(RATE_X2, ptr, 0, 0, 0);
	ptr++;
	fpga_write_cmd(RATE_Y2, ptr, 1, 1, 0);
	ptr++;
	fpga_write_cmd(RATE_Z2, ptr, 1, 1, 1);
	ptr++;
	fpga_write_cmd(ACC_X2, ptr, 1, 1, 2);
	ptr++;
	fpga_write_cmd(ACC_Y2, ptr, 1, 1, 3);
	ptr++;
	fpga_write_cmd(ACC_Z2, ptr, 1, 1, 4);
	ptr++;
	fpga_write_cmd(TEMP, ptr, 1, 1, 5);
	ptr++;
	fpga_write_cmd(STAT_SUM, ptr, 1, 1, 6);
	ptr++;
	fpga_write_cmd(STAT_SUM_SAT, ptr, 1, 0, 0);
	ptr++;
	fpga_write_cmd(STAT_COM, ptr, 1, 0, 1);
	ptr++;
	fpga_write_cmd(STAT_RATE_COM, ptr, 1, 0, 2);
	ptr++;
	fpga_write_cmd(STAT_RATE_X, ptr, 1, 0, 3);
	ptr++;
	fpga_write_cmd(STAT_RATE_Y, ptr, 1, 0, 4);
	ptr++;
	fpga_write_cmd(STAT_RATE_Z, ptr, 1, 0, 5);
	ptr++;
	fpga_write_cmd(STAT_ACC_X, ptr, 1, 0, 6);
	ptr++;
	fpga_write_cmd(STAT_ACC_Y, ptr, 1, 0, 7);
	ptr++;
	fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 8);
	ptr++;
	fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 9);
	ptr++;
	fifo_cmd_num = ptr;
}

static void sch16t_gen_cmd(uint32_t addr, uint8_t *buff, uint8_t read_valid, uint8_t is_sensor, uint8_t offset)
{
	buff[0] = read_valid ? 0x80 : 0x00;
	buff[0] |= is_sensor ? 0x40 : 0x00;
	buff[0] |= (offset & 0xf) << 2;
	buff[0] |= (offset >> 2) & 0x3;
	buff[1] = (offset & 3) << 6;
	_register_read(addr, buff + 2);
}

void SCH16T_FIFO::fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset)
{
	uint8_t buff[16];
	sch16t_gen_cmd(addr, buff, read_valid, is_sensor, offset);
	fpga_write_ram8(Addr_W8_SensorReadCmdRam8, ptr << 3, buff, 8);
}


bool SCH16T_FIFO::collect_and_publish()
{
	bool success = read_data();
	return success;
}

bool SCH16T_FIFO::read_data()
{
	hrt_abstime timestamp_sample = now;
	uint8_t pkt_num;
	uint8_t buff[32];

	pkt_num = fpga_read_reg8(Addr_RW8_SensorFifoCtrl);

	if (pkt_num == 0) {
		perf_count(_fifo_empty_perf);

	} else {
		// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure

		if (pkt_num > _fifo_gyro_samples) {
			// grab desired number of samples, but reschedule next cycle sooner
			int extra_samples = pkt_num - _fifo_gyro_samples;
			pkt_num = _fifo_gyro_samples;

			if (_fifo_gyro_samples > extra_samples) {
				// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
				const uint32_t reschedule_delay_us = (_fifo_gyro_samples - extra_samples) * static_cast<int>(FIFO_SAMPLE_DT);
				ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

			} else {
				// otherwise reschedule to run immediately
				ScheduleOnInterval(_fifo_empty_interval_us);
			}

		} else if (pkt_num < _fifo_gyro_samples) {
			// reschedule next cycle to catch the desired number of samples
			ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - pkt_num) * static_cast<int>(FIFO_SAMPLE_DT));
		}
	}

	bool success = false;

	if (pkt_num == _fifo_gyro_samples) {
		sensor_gyro_fifo_s gyro{};
		gyro.timestamp_sample = timestamp_sample;
		gyro.samples = pkt_num;
		gyro.dt = FIFO_SAMPLE_DT;

		sensor_accel_fifo_s accel{};
		accel.timestamp_sample = timestamp_sample;
		accel.samples = pkt_num;
		accel.dt = FIFO_SAMPLE_DT;
		// 18-bits of accelerometer data
		bool acc_scale_20bit = false;

		// 18-bits of 	// 20-bits of gyroscope data data
		bool gyro_scale_20bit = false;

		float temperature_sum{0};

		SensorData data = {};

		for (int i = 0; i < pkt_num; i++) {
			fpga_read_fifo8(Addr_W8_SensorValueFifo8, buff, 24);
			fpga_write_reg8(Addr_RW8_SensorFifoCtrl, 0x01);

			if (buff[0] & 0x80) { data.gyro_x = SPI32BITCONVERT2_20BIT(((buff[1] & 0xf) << 16) | (buff[2] << 8) | buff[3]); }

			if (buff[0] & 0x40) { data.gyro_y = -SPI32BITCONVERT2_20BIT(((buff[4] & 0xf) << 16) | (buff[5] << 8) | buff[6]); }

			if (buff[0] & 0x20) { data.gyro_z = -SPI32BITCONVERT2_20BIT(((buff[7] & 0xf) << 16) | (buff[8] << 8) | buff[9]); }

			if (buff[0] & 0x10) { data.acc_x = SPI32BITCONVERT2_20BIT(((buff[10] & 0xf) << 16) | (buff[11] << 8) | buff[12]); }

			if (buff[0] & 0x08) { data.acc_y = -SPI32BITCONVERT2_20BIT(((buff[13] & 0xf) << 16) | (buff[14] << 8) | buff[15]); }

			if (buff[0] & 0x04) { data.acc_z = -SPI32BITCONVERT2_20BIT(((buff[16] & 0xf) << 16) | (buff[17] << 8) | buff[18]); }

			if (buff[0] & 0x02) { data.temp = SPI32BITCONVERT2_20BIT(((buff[19] & 0xf) << 16) | (buff[20] << 8) | buff[21]) >> 4;}



			temperature_sum += data.temp;


			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_accel = INT16_MAX;
			static constexpr int16_t min_accel = INT16_MIN;

			if (data.acc_x >= max_accel || data.acc_x <= min_accel) {
				acc_scale_20bit = true;
			}

			if (data.acc_y >= max_accel || data.acc_y <= min_accel) {
				acc_scale_20bit = true;
			}

			if (data.acc_z >= max_accel || data.acc_z <= min_accel) {
				acc_scale_20bit = true;
			}



			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_gyro = INT16_MAX;
			static constexpr int16_t min_gyro = INT16_MIN;

			if (data.gyro_x >= max_gyro || data.gyro_x <= min_gyro) {
				gyro_scale_20bit = true;
			}

			if (data.gyro_y >= max_gyro || data.gyro_y <= min_gyro) {
				gyro_scale_20bit = true;
			}

			if (data.gyro_z >= max_gyro || data.gyro_z <= min_gyro) {
				gyro_scale_20bit = true;
			}

			gyro.x[i] = data.gyro_x;
			gyro.y[i] = data.gyro_y;
			gyro.z[i] = data.gyro_z;

			accel.x[i] = data.acc_x;
			accel.y[i] = data.acc_y;
			accel.z[i] = data.acc_z;

		}

		if (!acc_scale_20bit) {
			_px4_accel.set_scale(1.f / 3200.f); // 3200 LSB/(m/s2)

		} else {
			// 20 bit data scaled to 16 bit (2^4)
			for (int i = 0; i < pkt_num; i++) {
				// 20 bit hires mode
				// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
				// Accel data is 18 bit ()
				int16_t accel_x = (((buff[10] & 0xf) << 12) | (buff[11] << 4) | (buff[12] >> 4));
				int16_t accel_y = -(((buff[13] & 0xf) << 12) | (buff[14] << 4) | (buff[15] >> 4));
				int16_t accel_z = -(((buff[16] & 0xf) << 12) | (buff[17] << 4) | (buff[18] >> 4));

				accel.x[i] = accel_x;
				accel.y[i] = accel_y;
				accel.z[i] = accel_z;
			}

			_px4_accel.set_scale(1.f / 200.f); // 200 LSB/(m/s2)
		}

		// correct frame for publication
		for (int i = 0; i < accel.samples; i++) {
			// sensor's frame is +x forward, +y left, +z up
			//  flip y & z to publish right handed with z down (x forward, y right, z down)
			accel.x[i] = accel.x[i];
			accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MIN : accel.y[i];
			accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MIN : accel.z[i];
		}

		if (!gyro_scale_20bit) {
			_px4_gyro.set_scale(math::radians(1.f / 100.f));     // 100 LSB/(°/sec)

		} else {
			// 20 bit data scaled to 16 bit (2^4)
			for (int i = 0; i < pkt_num; i++) {
				gyro.x[i] = (((buff[1] & 0xf) << 12) | (buff[2] << 4) | (buff[3] >> 4));
				gyro.y[i] = -(((buff[4] & 0xf) << 12) | (buff[5] << 4) | (buff[6] >> 4));
				gyro.z[i] = -(((buff[7] & 0xf) << 12) | (buff[8] << 4) | (buff[9] >> 4));
			}

			_px4_gyro.set_scale(math::radians(16.f / 100.f));
		}

		// correct frame for publication
		for (int i = 0; i < gyro.samples; i++) {
			// sensor's frame is +x forward, +y left, +z up
			//  flip y & z to publish right handed with z down (x forward, y right, z down)
			gyro.x[i] = gyro.x[i];
			gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MIN : gyro.y[i];
			gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MIN : gyro.z[i];
		}

		const float temperature_avg = temperature_sum / pkt_num;

		_px4_gyro.updateFIFO(gyro);

		_px4_accel.updateFIFO(accel);

		_px4_accel.set_temperature(temperature_avg / 100.f); // Temperature signal sensitivity is 100 LSB/°C
		_px4_gyro.set_temperature(temperature_avg / 100.f);

		success = true;
	}

	return success;
}

void SCH16T_FIFO::reset_chip()
{
#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
	palClearLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
	hal.scheduler->delay(2000);
	palSetLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
#else
	register_write(CTRL_RESET, SPI_SOFT_RESET);
#endif
}

bool SCH16T_FIFO::read_product_id()
{
	uint32_t comp_id = 0, asic_id = 0;
	register_read(COMP_ID, 0);
	register_read(ASIC_ID, &comp_id);
	register_read(ASIC_ID, &asic_id);
	bool success = asic_id == 0x21 && comp_id == 0x21;

	return success;
}

void SCH16T_FIFO::configure_registers()
{
	uint32_t reg_value;

	for (auto &r : _registers) {
		register_write(r.addr, r.value);
	}

	register_read(CTRL_USER_IF, 0);
	register_read(CTRL_USER_IF, &reg_value);
	reg_value |= DRY_DRV_EN;
	register_write(CTRL_USER_IF, reg_value);
	register_write(CTRL_MODE, EN_SENSOR);
}

bool SCH16T_FIFO::validate_sensor_status()
{
	auto &s = _sensor_status;
	uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

	for (auto v : values) {
		if (v != 0xFFFF) {
			return false;
		}
	}

	return true;
}
bool SCH16T_FIFO::validate_register_configuration()
{
	uint32_t value;
	bool success = true;

	for (auto &r : _registers) {
		register_read(r.addr, 0); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
		register_read(r.addr, &value);

		if (value != r.value) {
			success = false;
		}
	}

	return success;
}



uint16_t SCH16T_FIFO::fpga_read_reg16(uint16_t reg)
{
	uint8_t wbuf[8];
	uint8_t ptr = 0;

	wbuf[ptr++] = FuncBit_Read | FuncBit_Reg | FuncBit_Bit16;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = 0xff;
	wbuf[ptr++] = 0xff;
	wbuf[ptr++] = 0xff;
	transfer(wbuf, wbuf, ptr);
	return wbuf[4] | (wbuf[5] << 8);
}

uint8_t SCH16T_FIFO::wait_direct_busy(void)
{
	uint16_t timeout = 0x200;

	while (timeout) {
		if (!(fpga_read_reg16(Addr_RW16_DirectSpiCtrl) & 1)) { return 1; }

		timeout--;
	}

	return 0;
}

uint16_t SCH16T_FIFO::_sch16t_read_status(uint16_t addr)
{
	uint32_t value;

	if (register_read(addr, &value) == 0) {
		return 0;
	}

	return value & 0xffff;
}

void SCH16T_FIFO::read_status_registers()
{
	_sch16t_read_status(STAT_SUM);
	_sensor_status.summary      = _sch16t_read_status(STAT_SUM);
	_sensor_status.summary      = _sch16t_read_status(STAT_SUM_SAT);
	_sensor_status.saturation   = _sch16t_read_status(STAT_COM);
	_sensor_status.common       = _sch16t_read_status(STAT_RATE_COM);
	_sensor_status.rate_common  = _sch16t_read_status(STAT_RATE_X);
	_sensor_status.rate_x       = _sch16t_read_status(STAT_RATE_Y);
	_sensor_status.rate_y       = _sch16t_read_status(STAT_RATE_Z);
	_sensor_status.rate_z       = _sch16t_read_status(STAT_ACC_X);
	_sensor_status.acc_x        = _sch16t_read_status(STAT_ACC_Y);
	_sensor_status.acc_y        = _sch16t_read_status(STAT_ACC_Z);
	_sensor_status.acc_z        = _sch16t_read_status(STAT_ACC_Z);
}

uint8_t SCH16T_FIFO::register_read(uint16_t addr, uint32_t *value)
{
	uint8_t tbuf[8];
	uint8_t rbuf[8];

	direct_mode = CtrlMode_Direct;
	sensor_ctrl(0);

	_register_read(addr, tbuf);

	wait_direct_busy();
	fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
	fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
	fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
	wait_direct_busy();
	fpga_read_ram8(Addr_RW8_DirectRam8, 0, rbuf, 6);

	if (value == 0) {
		return 0;
	}

	if (gen_crc8(rbuf) != rbuf[5]) {
		return 0;
	}

	*value = ((rbuf[2] & 0xf) << 16) | (rbuf[3] << 8) | (rbuf[4]);
	return 1;
}

// Non-data registers are the only writable ones and are 16 bit or less
void SCH16T_FIFO::register_write(uint16_t addr, uint32_t value)
{
	uint8_t tbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	direct_mode = CtrlMode_Direct;
	sensor_ctrl(0);

	_register_write(addr, value, tbuf);

	wait_direct_busy();
	fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
	fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
	fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
	wait_direct_busy();
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t SCH16T_FIFO::transfer_spi_frame(uint64_t frame)
{
	uint16_t buf[3];

	for (int index = 0; index < 3; index++) {
		uint16_t lower_byte = (frame >> (index << 4)) & 0xFF;
		uint16_t upper_byte = (frame >> ((index << 4) + 8)) & 0xFF;
		buf[3 - index - 1] = (lower_byte << 8) | upper_byte;
	}

	transferhword(buf, buf, 3);

	uint64_t value = {};

	for (int index = 0; index < 3; index++) {
		uint16_t lower_byte = buf[index] & 0xFF;
		uint16_t upper_byte = (buf[index] >> 8) & 0xFF;
		value |= (uint64_t)(upper_byte | (lower_byte << 8)) << ((3 - index - 1) << 4);
	}

	return value;
}
uint8_t SCH16T_FIFO::calculate_crc8(uint64_t frame)
{
	uint64_t data = frame & 0xFFFFFFFFFF00LL;
	uint8_t crc = 0xFF;

	for (int i = 47; i >= 0; i--) {
		uint8_t data_bit = data >> i & 0x01;
		crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
	}

	return crc;
}
void SCH16T_FIFO::fpga_read_fifo8(uint16_t reg, uint8_t *value, uint8_t size)
{
	uint8_t wbuf[32];
	uint8_t ptr = 0;
	uint8_t i;

	if (value == nullptr) { return; }

	wbuf[ptr++] = FuncBit_Read | FuncBit_Fifo | FuncBit_Bit8;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = size;
	wbuf[ptr++] = 0xff;

	for (i = 0; i < size && ptr < 32; i++) {
		wbuf[ptr++] = *(value + i);
	}

	transfer(wbuf, wbuf, ptr);

	for (i = 0; i < size; i++) {
		*(value + i) = wbuf[5 + i];
	}
}

void SCH16T_FIFO::fpga_write_reg8(uint16_t reg, uint8_t value)
{
	uint8_t wbuf[8];
	uint8_t ptr = 0;

	wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit8;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = value;

	transfer(wbuf, wbuf, ptr);
}
uint8_t SCH16T_FIFO::fpga_read_reg8(uint16_t reg)
{
	uint8_t wbuf[8];
	uint8_t ptr = 0;

	wbuf[ptr++] = FuncBit_Read | FuncBit_Reg | FuncBit_Bit8;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = 0xff;
	wbuf[ptr++] = 0xff;
	transfer(wbuf, wbuf, ptr);
	return wbuf[4];
}

void SCH16T_FIFO::fpga_write_reg16(uint16_t reg, uint16_t value)
{
	uint8_t wbuf[8];
	uint8_t ptr = 0;

	wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit16;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = value & 0xff;
	wbuf[ptr++] = (value >> 8) & 0xff;


	transfer(wbuf, wbuf, ptr);
}

void SCH16T_FIFO::fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size)
{
	uint8_t wbuf[32];
	uint8_t ptr = 0;
	uint8_t i;

	if (value == nullptr) { return; }

	wbuf[ptr++] = FuncBit_Write | FuncBit_Ram | FuncBit_Bit8;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = ram_addr & 0xff;
	wbuf[ptr++] = (ram_addr >> 8) & 0xff;

	for (i = 0; i < size && ptr < 32; i++) {
		wbuf[ptr++] = *(value + i);
	}

	transfer(wbuf, wbuf, ptr);

}

void SCH16T_FIFO::fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size)
{
	uint8_t wbuf[32];
	uint8_t ptr = 0;
	uint8_t i;

	if (value == nullptr) { return; }

	wbuf[ptr++] = FuncBit_Read | FuncBit_Ram | FuncBit_Bit8;
	wbuf[ptr++] = reg & 0xff;
	wbuf[ptr++] = (reg >> 8) & 0xff;
	wbuf[ptr++] = ram_addr & 0xff;
	wbuf[ptr++] = (ram_addr >> 8) & 0xff;
	wbuf[ptr++] = 0xff;

	for (i = 0; i < size && ptr < 32; i++) {
		wbuf[ptr++] = *(value + i);
	}

	transfer(wbuf, wbuf, ptr);

	for (i = 0; i < size; i++) {
		*(value + i) = wbuf[6 + i];
	}
}

uint8_t SCH16T_FIFO::fpga_test(void)
{
	uint16_t wreg, rreg = 0;
	wreg = rand() & 0xffff;
	fpga_write_reg16(Addr_RW16_TestReg, wreg);
	rreg = fpga_read_reg16(Addr_RW16_TestReg);

	if (wreg != rreg) {
		return 0;
	}

	return 1;
}
