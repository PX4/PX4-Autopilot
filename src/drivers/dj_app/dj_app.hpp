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
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

class RS485 : public ModuleBase<RS485>, public OutputModuleInterface
{
public:
	RS485();
	~RS485() override;

	static int task_spawn(int argc, char *argv[]);	// 이 코드가 Work Queue에 올라갈 때 한 번 실행되는 함수
	static int custom_command(int argc, char *argv[]);	// 코드 기능에 영향 x
	static int print_usage(const char *reason = nullptr);	// 코드 기능에 영향 x
	int print_status() override;				// 코드 기능에 영향 x

	void Run() override;	// 실질적으로 Work Queue에서 계속 돌아가는 함수

	// 모터 출력을 업데이트하는 함수(Run 함수에서는 이 함수를 실행하지 않는데 어떻게 실행되는지 의문, 아마 밑의 _mixing_output을 업데이트할 때 실행되는 듯?)
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	struct RTU	// Modbus RTU 통신에서 주고받는 패킷을 구조체로 추상화(일단은 단일 레지스터, 단일 데이터만 주고받을 수 있는 패킷)
	{
		uint8_t _device_address;
		uint8_t _function_code;
		uint8_t _register_address_high;
		uint8_t _register_address_low;
		uint8_t _data[2];
		uint8_t _crc_high;
		uint8_t _crc_low;
	};
	RTU _rtu_front{0x01};	// 앞바퀴 모터드라이버(로봇 진행방향 기준 왼쪽 모터드라이버)
	RTU _rtu_back{0x02};	// 뒷바퀴 모터드라이버(로봇 진행방향 기준 오른쪽 모터드라이버)

	enum class Mode : uint16_t	// 모터드라이버 모드 추상화(위치 모드는 사용 안해서 뺌)
	{
		None = 0,
		Velocity = 3,
		Torque = 4
	};

	enum class RegisterAddr : uint16_t
	{
    		// Initial Setting //
    		/////////////////////
			RS485_NODE_ID = 0x2001,
    		OPR_MODE = 0x200D,
    		CONTROL_REG = 0x200E,
    		L_RATED_CUR = 0x2033,
    		L_MAX_CUR = 0x2034,
    		R_RATED_CUR = 0x2063,
    		R_MAX_CUR = 0x2064,
    		/////////////////////

    		// Contorl Parameters //
    		////////////////////////
			SAVE_EEPROM = 0x2010,
    		MOTOR_MAX_RPM = 0x2008,
    		L_CMD_RPM = 0x2088,
    		R_CMD_RPM = 0x2089,
    		L_CMD_TOQ = 0x2090,
    		R_CMD_TOQ = 0x2091,
    		////////////////////////

    		// Read Only //
    		///////////////
    		DRIVER_VOL = 0x20A1,
    		DRIVER_TEMP = 0x20B0,
    		L_FB_RPM = 0x20AB,
    		R_FB_RPM = 0x20AC,
    		L_FB_TOQ = 0x20AD,
    		R_FB_TOQ = 0x20AE
    		///////////////
	};

	enum class CMD : uint16_t
	{
    		EMER_STOP = 0x0005,
    		ALRM_CLR = 0x0006,
    		ENABLE = 0x0008
	};

	Mode _motor_mode_front = Mode::None;	// 모터드라이버 모드 값 변수(앞바퀴 모터드라이버)
	Mode _motor_mode_back = Mode::None;	// 모터드라이버 모드 값 변수(뒷바퀴 모터드라이버)

	bool _motor_enabled_front = false;	// 모터드라이버 enabled 되었는지 저장하는 변수(앞바퀴 모터드라이버)
	bool _motor_enabled_back = false;	// 모터드라이버 enabled 되었는지 저장하는 변수(뒷바퀴 모터드라이버)

	void setMotorSpeed(RTU* rtu, uint16_t rpm, bool side);	// rpm과 Left, Right(모터드라이버 기준)를 매개변수로 받아 속도 설정
	void readEncoder();		// 엔코더 읽기(미구현)
	void setMotorMode(RTU* rtu, Mode mode);	// 모터드라이버 모드 설정
	void setMotorEnabled(RTU *rtu);			// 모터드라이버 enable로 설정
	void setDeviceAddress(RTU* rtu, uint16_t address);	// 모터드라이버 디바이스 주소 설정하는 함수(일반적으로는 안 씀)

private:
	// 아마 얘가 실질적으로 모터를 돌리는 인스턴스로 추측된다.
	MixingOutput _mixing_output{"DJ", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	const char* _port_name = "/dev/ttyS3";	// 디바이스 포트 이름(pixhawk UART 포트이름이 dev/ttyS3)
	const int _baudrate = B115200;		// baudrate(모터드라이브의 default가 115200)
	int _rs485_fd = 0;			// rs485 통신 디스크립터
	bool _rs485_initialized = false;	// rs485 통신이 초기화되었는지를 저장하는 bool 변수

	// uORB 토픽 선언(일단 roboclaw에 있는 거 넣었는데 필수인지는 모르겠음. 추후 테스트 예정)
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::PublicationData<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	ssize_t initializeRS485();	// RS485 통신 초기화
	void setRTUPacket(RTU* rtu, uint8_t device_address, uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length); // 원하는 값으로 패킷 설정
	uint16_t calculateCRC(uint8_t *data, size_t data_length);	// CRC 계산
	void writeData(RTU rtu);	// 데이터 쓰기
	uint16_t readData(RTU rtu);	// 데이터 읽기(미구현)

	// 왜인지 모르겠는데 pwm_out에서 쓰길래 추가함(나중에 빼고 테스트해볼 예정)
	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
