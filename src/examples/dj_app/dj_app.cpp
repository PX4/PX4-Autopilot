#include "dj_app.hpp"
#include <stdio.h>

RS485::RS485() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::ttyS3)
{
	PX4_INFO("Instance has constructed!\n");
}

RS485::~RS485()
{
	close(_rs485_fd);	// rs485 디스크립터를 닫는다.
	PX4_INFO("Instance has destructed...\n");
}

int RS485::task_spawn(int argc, char *argv[])
{
	RS485 *instance = new RS485();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int RS485::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RS485::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("dj_app", "example");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int RS485::print_status()
{
	_mixing_output.printStatus();
	return 0;
}

bool RS485::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	return 0;
}

void RS485::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

ssize_t RS485::initializeRS485()
{
	_rs485_fd = open(_port_name, O_RDWR | O_NOCTTY);	// rs485 디스크립터를 연다.
	if (_rs485_fd < 0) return ERROR;

	ssize_t ret = 0;
	struct termios rs485_config {};			// termios 구조체 초기화
	ret = tcgetattr(_rs485_fd, &rs485_config);	// 현재 터미널 설정 termios 구조체에 가져옴
	if (ret < 0) return ERROR;

	rs485_config.c_cflag &= ~PARENB;	// Parity bit None
	rs485_config.c_cflag &= ~CSTOPB;	// Stop bit 1로 설정
	rs485_config.c_cflag &= ~CSIZE;		// Data bits 설정 초기화
	rs485_config.c_cflag |= CS8;		// Data bits 8로 설정
	rs485_config.c_cflag &= ~CRTSCTS;	// Flow Conotrol 끄기
	rs485_config.c_cflag |= CREAD | CLOCAL;
	rs485_config.c_oflag &= ~OPOST;
	rs485_config.c_oflag &= ~ONLCR;
	cfsetispeed(&rs485_config, B115200);
	cfsetospeed(&rs485_config, B115200);

	tcflush(_rs485_fd, TCIFLUSH);
	ret = tcsetattr(_rs485_fd, TCSANOW, &rs485_config);	// termios 구조체 설정을 디스크립터에 저장
	if (ret < 0) return ERROR;

	PX4_INFO("Successfully connected!\n");
	return OK;
}

ssize_t RS485::setMotorSpeed(uint16_t rpm, bool side)
{
	if (_motor_mode != Mode::Velocity) setMotorMode(Mode::Velocity);
	if (side == 0) setRTUPacket(0x06, 0x2088, (uint8_t*)&rpm, sizeof(rpm));
	if (side == 1) setRTUPacket(0x06, 0x2089, (uint8_t*)&rpm, sizeof(rpm));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	return writeData();
}

ssize_t RS485::readEncoder()
{
	uint16_t register_number = 0x0001;

	if (_motor_mode != Mode::Velocity) setMotorMode(Mode::Velocity);
	setRTUPacket(0x03, 0x20AB, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	speed_left = readData();
	setRTUPacket(0x03, 0x20AC, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	speed_right = readData();

	setMotorMode(Mode::AbsolutePosition);
	setRTUPacket(0x03, 0x20A7, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	position_left = readData();
	setRTUPacket(0x03, 0x20A8, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	position_left = (position_left << 8) | readData();

	setRTUPacket(0x03, 0x20A9, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	position_right = readData();
	setRTUPacket(0x03, 0x20AA, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	position_right = (position_right << 8) | readData();

	wheel_encoders_s wheel_encoders{};
	wheel_encoders.wheel_speed[0] = speed_right;
	wheel_encoders.wheel_speed[1] = speed_left;
	wheel_encoders.wheel_angle[0] = position_right;
	wheel_encoders.wheel_angle[1] = position_left;
	wheel_encoders.timestamp = hrt_absolute_time();
	_wheel_encoders_pub.publish(wheel_encoders);

	printf("speed_right : %.2x\n", speed_right);
	printf("speed_left : %.2x\n", speed_left);
	printf("position_right : %.4lx\n", position_right);
	printf("position_left : %.4lx\n", position_left);


	return OK;
}

ssize_t RS485::setMotorMode(Mode mode)
{
	setRTUPacket(0x06, 0x200D, (uint8_t*)&mode, sizeof(mode));
	calculateCRC((uint8_t*)&_rtu, sizeof(_rtu) - 2);
	_motor_mode = mode;
	return writeData();
}

uint16_t RS485::calculateCRC(const uint8_t *data, size_t data_length)
{
	const uint16_t polynomial = 0xA001;
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < data_length; ++i)
	{
		crc ^= data[i];
		for (int j = 0; j < 8; ++j)
        	{
			uint16_t LSB = crc & 0x0001;
			crc >>= 1;
			if (LSB) crc ^= polynomial;
        	}
	}
	_rtu._crc_high = crc & 0xFF;
	_rtu._crc_low = crc >> 8;
	return crc;
}

void RS485::setRTUPacket(uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length)
{
	_rtu._function_code = function_code;
	_rtu._register_address_high = register_address >> 8;
	_rtu._register_address_low = register_address & 0xFF;
	for (size_t i = 0; i < data_length; i++)
		_rtu._data[i] = data[data_length - 1 - i];
}

void RS485::printRTUPacket(void)
{
	PX4_INFO("%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x\n",
	_rtu._device_address, _rtu._function_code, _rtu._register_address_high, _rtu._register_address_low
	, _rtu._data[0], _rtu._data[1], _rtu._crc_high, _rtu._crc_low);
}

ssize_t RS485::writeData()
{
	ssize_t ret = write(_rs485_fd, &_rtu, sizeof(_rtu));
	printRTUPacket();
	usleep(4000);
	return ret;
}

uint16_t RS485::readData()
{
	uint16_t buf;
	write(_rs485_fd, &_rtu, sizeof(_rtu));
	printRTUPacket();
	read(_rs485_fd, &buf, 2);
	return buf;
}

extern "C" __EXPORT int dj_app_main(int argc, char *argv[])
{
	// PX4_INFO("DJ has started!\n");

	// int quit;
	// RS485 rs485;
	// rs485.initializeRS485();
	// while(quit)
	// {
	// 	scanf("%d", &quit);
	// 	if (quit == 1)
	// 	{
	// 		rs485.setMotorSpeed(100, 0);
	// 		rs485.setMotorSpeed(100, 1);
	// 	}
	// 	else if (quit == 2)
	// 	{
	// 		rs485.setMotorSpeed(0, 0);
	// 		rs485.setMotorSpeed(0, 1);
	// 	}
	// 	else if (quit == 3)
	// 	{
	// 		rs485.readEncoder();
	// 	}

	// }
	// return OK;
	printf("Hello");
	return RS485::main(argc, argv);
}
