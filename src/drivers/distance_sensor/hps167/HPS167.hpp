/**
 * @file hps167.cpp
 * @author limshoonkit<lsk950329@hotmail.com>
 *
 * Driver for the Hypersen HPS167 Time-of-Flight (TOF) distance sensor
 * for the serial interface. Make sure to disable MAVLINK messages
 * (MAV_0_CONFIG PARAMETER) on the serial port you connect the sensor,i.e TELEM2.
 *
 */


#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t HPS167_MEASURE_INTERVAL{20_ms};	// 50hz (20ms) sensor data rate

/* Frame start delimiter */
static constexpr unsigned char START_BYTE{0x0A};

/**
 * CMD_ACQUIRE_SENSOR_INFO - Acquire the sensor information
 * After the sensor is powered up, system automatically performs the initialization procedures and the serial
 * interface will output “Hypersen” if the initialization succeeded. A start byte “0x0A” is used to indicate the start
 * of each command and returned data frame. Each HPS-167 has its universally unique identifier (UUID),
 * which can be read out by sending a command from the host.
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x2E     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0xFC 0x3C
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Ack (1B) | UUID (16B) | Production Data (3B) | Ver (2B)    | CRC (2B)
 * | 0x0A       |  0x18     | 0xB0     | ...        | Year Month Day       | Major Minor | CRC data frame (Byte 2 to Byte 23)
 *
 * Example Decoding:
 * 0x0A: Start byte
 * 0x18: Data length (24 byte data)
 * 0xB0: Acknowledge
 * 0x52 0x13 0x29 0x8C 0xC7 0xE0 0xE5 0x11 0x8D 0x2B 0xB9 0x57 0x2C 0xF3 0xAD 0x25: UUID
 * 0x10 0x0A 0x08: 16/10/08
 * 0x01 0x09: Ver. 1.9
 * 0x22 0xE9: CRC16-CCITT MSB and LSB byte
 */
static constexpr uint8_t CMD_ACQUIRE_SENSOR_INFO[]={0x0A, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3C};

/**
 * CMD_CONTINUOUS_RANGING
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x24     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0x0F 0x72
 *
 * CMD_SINGLE_RANGING
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x22     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0xAE 0x57
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Reserved (3B) | Distance (2B) | Magnitude (3B) | Ambient (1B) | Precision (2B) | CRC (2B)
 * | 0x0A       |  0x0D     | ...           | MSB LSB       | MSB LSB Exp.   | ...          | MSB LSB        | CRC data frame (Byte 2 to Byte 12)
 *
 * Example Decoding:
 * 0x0A: Start byte
 * 0x0D: Data length (13 byte data)
 * Distance = (0x06 * 256 + 0xD9) / 1000.0f = 1.753 (unit: m)
 * Magnitude = ((0xFC * 256 + 0x8C) << 0x02) / 10000.0f = 25.8608
 * Ambient ADC, Relative ambient IR intensity = 1
 * Precision, small values correspond to small measurement erroes = (0x00 * 256) + 0x01 = 1
 * 0x9B 0x94: CRC16-CCITT MSB and LSB byte
 *
 * Note: Sensor will output a 65.53m over range indication if the measurement result is over ranged or
 * receiving signal is too low.
 */
static constexpr uint8_t CMD_CONTINUOUS_RANGING[]={0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72};
static constexpr uint8_t CMD_SINGLE_RANGING[]={0x0A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x57};
static constexpr uint8_t DISTANCE_MSB_POS{5};
static constexpr uint8_t DISTANCE_LSB_POS{6};

/**
 * CMD_STOP_RANGING
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x30     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0xBC 0x6F
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Ack (1B) | CRC (2B)
 * | 0x0A       |  0x03     | ...      | CRC data frame (Byte 2)
 *CMD_STOP_RANGING
 * ACK = 0x01: Succeed; 0x00: Fail
 */
static constexpr uint8_t CMD_STOP_RANGING[]={0x0A, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x6F};

/**
 * CMD_OFFSET_COMPENSATION - Set offset compensation value
 * Due to the individual deviation of sensor performances, this command can be used to compensate the small
 * offset deviation to achieve higher ranging precision. The offset values will be automatically saved to the
 * internal non-volatile memory and reloaded with each power up.
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)                         |  CRC (2B)
 * | 0x0A       |  0x38     |  0x1A OffsetMSB OffsetLSB 0x00 0x00 0x00 |  CRC (Byte 0 to Byte 7)
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Ack (1B) | CRC (2B)
 * | 0x0A       |  0x03     | ...      | CRC data frame (Byte 2)
 *
 * ACK = 0x01: Succeed; 0x00: Fail
 *
 * Offset = Actual distance – Sensor measured distance, unit: mm
 * Example:
 * Actual distance: 200mm, sensor measured distance: 215mm
 * Offset = 200 – 215 = -15 = 0xFFF1 (Offset MSB = 0xFF, Offset LSB = 0xF1)
 */
static constexpr uint8_t CMD_OFFSET_COMPENSATION[]={0x0A, 0x38, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * CMD_LOAD_CFG_PROFILE - Load configuration profile
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)                  |  CRC (2B)
 * | 0x0A       |  0x3E     |  profile 0x00 0x00 0x00 0x00 0x00 |  CRC (Byte 0 to Byte 7)
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Ack (1B) | CRC (2B)
 * | 0x0A       |  0x03     | ...      | CRC data frame (Byte 2)
 *
 * ACK = 0x01: Succeed; 0x00: Fail
 * profile = 0x00: User; 0xFF: Factory
 */
static constexpr uint8_t CMD_LOAD_CFG_PROFILE[]={0x0A, 0x3E, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * CMD_OUTPUT_FILTER - Set output filter strength
 * Increasing this value will improve the stability of output data but sacrifice some sensitivity. Decreasing this
 * value makes the output data more sensitive to the distance change but sacrifice some stability. The default
 * value is “0x000” and the setting values within ±100 are recommended.
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)             |  CRC (2B)
 * | 0x0A       |  0x3D     |  0xAA MSB LSB 0x00 0x00 0x00 |  CRC (Byte 0 to Byte 7)
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Ack (1B) | CRC (2B)
 * | 0x0A       |  0x03     | ...      | CRC data frame (Byte 2)
 *
 * ACK = 0x01: Succeed; 0x00: Fail
 *
 * Example:
 * Decrease the output stability by 50 units -> Filter value = -50 = 0xFFCE (Filter MSB = 0xFF, Filter LSB=0xCE)
 * Increase the output stability by 50 units -> Filter value = 50 = 0x0032 (Filter MSB = 0x00, Filter LSB=0x32)
 */
static constexpr uint8_t CMD_OUTPUT_FILTER[]={0x0A, 0x3D, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * CMD_ACQUIRE_AFE_TEMP - Acquire the analog frontend (AFE) temperature
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)                |  CRC (2B)
 * | 0x0A       |  0x3F     |  0x01 0x00 0x00 0x00 0x00 0x00 |  0x36 0x86
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | AFE Temperature (2B) | CRC (2B)
 * | 0x0A       |  0x04     | MSB LSB              | CRC data frame (Byte 2 to Byte 3)
 */
static constexpr uint8_t CMD_ACQUIRE_AFE_TEMP[]={0x0A, 0x3F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x86};


class HPS167 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	HPS167(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~HPS167() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();

	PX4Rangefinder	_px4_rangefinder;

	char _port[20] {};

	int _file_descriptor{-1};

	uint8_t _linebuf[15] {};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
