#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

class RS485 : public ModuleBase<RS485>, public OutputModuleInterface
{
public:
	RS485();
	~RS485() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	void Run() override;

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	enum class Mode : uint16_t
	{
		None = 0,
		RelativePosition = 1,
		AbsolutePosition = 2,
		Velocity = 3,
		Torque = 4
	};

	ssize_t initializeRS485();	// RS485 통신 초기화
	ssize_t setMotorSpeed(uint16_t rpm, bool side);	// rpm과 왼,오른쪽을 매개변수로 받아 모터 속도 설정
	ssize_t readEncoder();

private:
	MixingOutput _mixing_output{"DJ_APP", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	typedef struct RTUPacket	// Modbus RTU 통신에서 주고받는 패킷을 구조체로 추상화
	{
		const uint8_t _device_address = 0x01;
		uint8_t _function_code;
		uint8_t _register_address_high;
		uint8_t _register_address_low;
		uint8_t _data[2];
		uint8_t _crc_high;
		uint8_t _crc_low;
	}RTU;
	RTU _rtu;

	const char* _port_name = "/dev/ttyS3";	// 디바이스 포트 이름(pixhawk UART 포트이름이 dev/ttyS3)
	const int _baudrate = B115200;		// baudrate(모터드라이브의 default가 115200)
	int _rs485_fd = 0;			// rs485 통신 디스크립터

	uint16_t speed_left = 0, speed_right = 0;
	uint32_t position_left = 0, position_right = 0;

	uORB::PublicationData<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	Mode _motor_mode = Mode::None;

	ssize_t setMotorMode(Mode mode);	// 모터드라이버 모드 설정
	uint16_t calculateCRC(const uint8_t *data, size_t data_length);	// CRC 계산
	void setRTUPacket(uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length);	// 원하는 값으로 패킷 설정
	void printRTUPacket(void);	// 현재 패킷에 저장된 데이터 출력(디버깅용)
	ssize_t writeData();	// 데이터 쓰기
	uint16_t readData();	// 데이터 읽기

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
