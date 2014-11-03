#include "RoboClaw.h"

// px4 added
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <systemlib/err.h> // errx
#include <unistd.h> // usleep

//
// Constructor
//
	RoboClaw::RoboClaw(const char *port, uint32_t tout, bool doack)
{
	timeout = tout;
	ack = doack;
	uart = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	// setup uart
	struct termios uart_config;
	int ret = tcgetattr(uart, &uart_config);

	if (ret < 0) { errx(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, B38400);

	if (ret < 0) { errx(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, B38400);

	if (ret < 0) { errx(1, "failed to set output speed"); }

	ret = tcsetattr(uart, TCSANOW, &uart_config);

	if (ret < 0) { errx(1, "failed to set attr"); }

	// setup uart polling
	uartPoll[0].fd = uart;
	uartPoll[0].events = POLLIN;
}

//
// Destructor
//
RoboClaw::~RoboClaw()
{
	close(uart);
}

bool RoboClaw::write(uint8_t byte)
{
	if (::write(uart, &byte, 1) < 0) {
		return false;

	} else {
		return true;
	}
}

uint8_t RoboClaw::read(uint32_t tout)
{
	uint8_t byte = 0;
	int pollrc = poll(uartPoll, 1, tout);

	if (pollrc > 0) {
		if (uartPoll[0].revents & POLLIN) {
			int ret = ::read(uart, &byte, 1);

			if (ret < 0) {
				byte = 0;
			}
		}

	} else if (pollrc < 0) {
		warnx("poll error");

	} else {
		warnx("poll timeout");
	}

	return byte;
}

void RoboClaw::HandleReadError()
{
	usleep(10000);
	tcflush(uart, TCIFLUSH);
}

bool RoboClaw::write_n(uint8_t cnt, ...)
{
	uint8_t crc = 0;

	//send data with crc
	va_list marker;
	va_start(marker, cnt);       /* Initialize variable arguments. */

	for (uint8_t index = 0; index < cnt; index++) {
		// px4 changes, uint16_t to int in va_arg
		uint8_t data = va_arg(marker, int);
		crc += data;
		write(data);
	}

	va_end(marker);                /* Reset variable arguments.      */

	if (ack) {
		write((crc & 0x7F) | 0x80);

	} else {
		write(crc & 0x7F);
	}

	if (ack)
		if (read(timeout) == 0xFF) {
			return true;
		}

	return false;
}

bool RoboClaw::ForwardM1(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M1FORWARD, speed);
}

bool RoboClaw::BackwardM1(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M1BACKWARD, speed);
}

bool RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage)
{
	return write_n(3, address, SETMINMB, voltage);
}

bool RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage)
{
	return write_n(3, address, SETMAXMB, voltage);
}

bool RoboClaw::ForwardM2(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M2FORWARD, speed);
}

bool RoboClaw::BackwardM2(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M2BACKWARD, speed);
}

bool RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M17BIT, speed);
}

bool RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
	return write_n(3, address, M27BIT, speed);
}

bool RoboClaw::ForwardMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDFORWARD, speed);
}

bool RoboClaw::BackwardMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDBACKWARD, speed);
}

bool RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDRIGHT, speed);
}

bool RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDLEFT, speed);
}

bool RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDFB, speed);
}

bool RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed)
{
	return write_n(3, address, MIXEDLR, speed);
}

bool RoboClaw::read_n(uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
	uint8_t crc;
	write(address);
	crc = address;
	write(cmd);
	crc += cmd;

	//send data with crc
	va_list marker;
	va_start(marker, cnt);       /* Initialize variable arguments. */

	for (uint8_t index = 0; index < cnt; index++) {
		// px4 changes, uint16_t to int in va_arg
		uint32_t *ptr = (uint32_t *)va_arg(marker, int);

		uint32_t value;
		uint8_t data = read(timeout);
		crc += data;
		value = (uint32_t)data << 24;

		data = read(timeout);
		crc += data;
		value |= (uint32_t)data << 16;

		data = read(timeout);
		crc += data;
		value |= (uint32_t)data << 8;

		data = read(timeout);
		crc += data;
		value |= (uint32_t)data;

		*ptr = value;
	}

	va_end(marker);                /* Reset variable arguments.      */

	uint8_t data = read(timeout);

	// px4 changes, flush input buffer if not valid data
	bool valid = ((crc & 0x7F) == data);

	if (!valid) { HandleReadError(); }

	return valid;
}

uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
	uint8_t crc;
	write(address);
	crc = address;
	write(cmd);
	crc += cmd;

	uint32_t value;
	uint8_t data = read(timeout);
	crc += data;
	value = (uint32_t)data << 24;

	data = read(timeout);
	crc += data;
	value |= (uint32_t)data << 16;

	data = read(timeout);
	crc += data;
	value |= (uint32_t)data << 8;

	data = read(timeout);
	crc += data;
	value |= (uint32_t)data;

	data = read(timeout);
	crc += data;

	if (status) {
		*status = data;
	}

	data = read(timeout);

	if (valid) {
		*valid = ((crc & 0x7F) == data);
	}

	// px4, added to flush buffer
	if (!valid) { HandleReadError(); }

	return value;
}

uint32_t RoboClaw::ReadEncM1(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM1ENC, status, valid);
}

uint32_t RoboClaw::ReadEncM2(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM2ENC, status, valid);
}

uint32_t RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM1SPEED, status, valid);
}

uint32_t RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM2SPEED, status, valid);
}

bool RoboClaw::ResetEncoders(uint8_t address)
{
	return write_n(2, address, RESETENC);
}

bool RoboClaw::ReadVersion(uint8_t address, char *version)
{
	uint8_t crc;
	write(address);
	crc = address;
	write(GETVERSION);
	crc += GETVERSION;

	for (uint8_t i = 0; i < 32; i++) {
		version[i] = read(timeout);
		crc += version[i];

		if (version[i] == 0) {
			if ((crc & 0x7F) == read(timeout)) {
				return true;

			} else {
				// px4, handle reading error
				HandleReadError();
				return false;
			}
		}
	}

	// px4, handle reading error
	HandleReadError();

	return false;
}

uint16_t RoboClaw::Read2(uint8_t address, uint8_t cmd, bool *valid)
{
	uint8_t crc;
	write(address);
	crc = address;
	write(cmd);
	crc += cmd;

	uint16_t value;
	uint8_t data = read(timeout);
	crc += data;
	value = (uint16_t)data << 8;

	data = read(timeout);
	crc += data;
	value |= (uint16_t)data;

	data = read(timeout);

	if (valid) {
		*valid = ((crc & 0x7F) == data);
	}

	// px4, handle reading error
	HandleReadError();

	return value;
}

uint16_t RoboClaw::ReadMainBatteryVoltage(uint8_t address, bool *valid)
{
	return Read2(address, GETMBATT, valid);
}

uint16_t RoboClaw::ReadLogicBattVoltage(uint8_t address, bool *valid)
{
	return Read2(address, GETLBATT, valid);
}

bool RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
	return write_n(3, address, SETMINLB, voltage);
}

bool RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
	return write_n(3, address, SETMAXLB, voltage);
}

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

bool RoboClaw::SetM1VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps)
{
	uint32_t kd = kd_fp * 65536;
	uint32_t kp = kp_fp * 65536;
	uint32_t ki = ki_fp * 65536;
	return write_n(18, address, SETM1PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

bool RoboClaw::SetM2VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps)
{
	uint32_t kd = kd_fp * 65536;
	uint32_t kp = kp_fp * 65536;
	uint32_t ki = ki_fp * 65536;
	return write_n(18, address, SETM2PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM1ISPEED, status, valid);
}

uint32_t RoboClaw::ReadISpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
	return Read4_1(address, GETM2ISPEED, status, valid);
}

bool RoboClaw::DutyM1(uint8_t address, uint16_t duty)
{
	return write_n(4, address, M1DUTY, SetWORDval(duty));
}

bool RoboClaw::DutyM2(uint8_t address, uint16_t duty)
{
	return write_n(4, address, M2DUTY, SetWORDval(duty));
}

bool RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2)
{
	return write_n(6, address, MIXEDDUTY, SetWORDval(duty1), SetWORDval(duty2));
}

bool RoboClaw::SpeedM1(uint8_t address, uint32_t speed)
{
	return write_n(6, address, M1SPEED, SetDWORDval(speed));
}

bool RoboClaw::SpeedM2(uint8_t address, uint32_t speed)
{
	return write_n(6, address, M2SPEED, SetDWORDval(speed));
}

bool RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2)
{
	return write_n(10, address, MIXEDSPEED, SetDWORDval(speed1), SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed)
{
	return write_n(10, address, M1SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

bool RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
	return write_n(10, address, M2SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}
bool RoboClaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2)
{
	return write_n(10, address, MIXEDSPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(speed2));
}

bool RoboClaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(11, address, M1SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(11, address, M2SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2,
				 uint32_t distance2, uint8_t flag)
{
	return write_n(19, address, MIXEDSPEEDDIST, SetDWORDval(speed2), SetDWORDval(distance1), SetDWORDval(speed2),
		       SetDWORDval(distance2), flag);
}

bool RoboClaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(15, address, M1SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(15, address, M2SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1,
				      uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	return write_n(23, address, MIXEDSPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(distance1),
		       SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

bool RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
	bool valid;
	uint16_t value = Read2(address, GETBUFFERS, &valid);

	if (valid) {
		depth1 = value >> 8;
		depth2 = value;
	}

	return valid;
}

bool RoboClaw::ReadCurrents(uint8_t address, uint8_t &current1, uint8_t &current2)
{
	bool valid;
	uint16_t value = Read2(address, GETCURRENTS, &valid);

	if (valid) {
		current1 = value >> 8;
		current2 = value;
	}

	return valid;
}

bool RoboClaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2)
{
	return write_n(18, address, MIXEDSPEED2ACCEL, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(accel2),
		       SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1,
					uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	return write_n(27, address, MIXEDSPEED2ACCELDIST, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(distance1),
		       SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

bool RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel)
{
	return write_n(6, address, M1DUTY, SetWORDval(duty), SetWORDval(accel));
}

bool RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel)
{
	return write_n(6, address, M2DUTY, SetWORDval(duty), SetWORDval(accel));
}

bool RoboClaw::DutyAccelM1M2(uint8_t address, int16_t duty1, uint16_t accel1, int16_t duty2, uint16_t accel2)
{
	// px4 changes, fixed typo MIXEDDUTY -> MIXEDDUTYACCEL
	return write_n(10, address, MIXEDDUTYACCEL, SetWORDval(duty1), SetWORDval(accel1), SetWORDval(duty2),
		       SetWORDval(accel2));
}

bool RoboClaw::ReadM1VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps)
{
	uint32_t Kp, Ki, Kd;
	bool valid = read_n(4, address, READM1PID, &Kp, &Ki, &Kd, &qpps);
	Kp_fp = ((float)Kp) / 65536;
	Ki_fp = ((float)Ki) / 65536;
	Kd_fp = ((float)Kd) / 65536;
	return valid;
}

bool RoboClaw::ReadM2VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps)
{
	uint32_t Kp, Ki, Kd;
	bool valid = read_n(4, address, READM2PID, &Kp, &Ki, &Kd, &qpps);
	Kp_fp = ((float)Kp) / 65536;
	Ki_fp = ((float)Ki) / 65536;
	Kd_fp = ((float)Kd) / 65536;
	return valid;
}

bool RoboClaw::SetMainVoltages(uint8_t address, uint16_t min, uint16_t max)
{
	return write_n(6, address, SETMAINVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool RoboClaw::SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max)
{
	return write_n(6, address, SETLOGICVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool RoboClaw::ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
	uint32_t value; // px4 change from uint16_t to uint32_t
	bool valid = read_n(1, address, GETMINMAXMAINVOLTAGES, &value);
	min = value >> 16;
	max = value & 0xFFFF;
	return valid;
}

bool RoboClaw::ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
	uint32_t value; // px4 change from uint16_t to uint32_t
	bool valid = read_n(1, address, GETMINMAXLOGICVOLTAGES, &value);
	min = value >> 16;
	max = value & 0xFFFF;
	return valid;
}

bool RoboClaw::SetM1PositionPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, float kiMax_fp,
				uint32_t deadzone, uint32_t min, uint32_t max)
{
	uint32_t kd = kd_fp * 1024;
	uint32_t kp = kp_fp * 1024;
	uint32_t ki = ki_fp * 1024;
	uint32_t kiMax = kiMax_fp * 1024;
	return write_n(30, address, SETM1POSPID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(kiMax),
		       SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

bool RoboClaw::SetM2PositionPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, float kiMax_fp,
				uint32_t deadzone, uint32_t min, uint32_t max)
{
	uint32_t kd = kd_fp * 1024;
	uint32_t kp = kp_fp * 1024;
	uint32_t ki = ki_fp * 1024;
	uint32_t kiMax = kiMax_fp * 1024;
	return write_n(30, address, SETM2POSPID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(kiMax),
		       SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

bool RoboClaw::ReadM1PositionPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, float &KiMax_fp,
				 uint32_t &DeadZone, uint32_t &Min, uint32_t &Max)
{
	uint32_t Kp, Ki, Kd, KiMax;
	bool valid = read_n(7, address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
	Kp_fp = ((float)Kp) / 1024;
	Ki_fp = ((float)Ki) / 1024;
	Kd_fp = ((float)Kd) / 1024;
	KiMax = ((float)KiMax_fp) / 1024;
	return valid;
}

bool RoboClaw::ReadM2PositionPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, float &KiMax_fp,
				 uint32_t &DeadZone, uint32_t &Min, uint32_t &Max)
{
	uint32_t Kp, Ki, Kd, KiMax;
	bool valid = read_n(7, address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
	Kp_fp = ((float)Kp) / 1024;
	Ki_fp = ((float)Ki) / 1024;
	Kd_fp = ((float)Kd) / 1024;
	KiMax = ((float)KiMax_fp) / 1024;
	return valid;
}

bool RoboClaw::SpeedAccelDeccelPositionM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel,
		uint32_t position, uint8_t flag)
{
	return write_n(19, address, M1SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel),
		       SetDWORDval(position), flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel,
		uint32_t position, uint8_t flag)
{
	return write_n(19, address, M2SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel),
		       SetDWORDval(position), flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM1M2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t deccel1,
		uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag)
{
	return write_n(35, address, MIXEDSPEEDACCELDECCELPOS, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(deccel1),
		       SetDWORDval(position1), SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(deccel2), SetDWORDval(position2), flag);
}

bool RoboClaw::ReadTemp(uint8_t address, uint16_t &temp)
{
	bool valid;
	temp = Read2(address, GETTEMP, &valid);
	return valid;
}

uint8_t RoboClaw::ReadError(uint8_t address, bool *valid)
{
	uint8_t crc;
	write(address);
	crc = address;
	write(GETERROR);
	crc += GETERROR;

	uint8_t value = read(timeout);
	crc += value;

	if (valid) {
		*valid = ((crc & 0x7F) == read(timeout));

	} else {
		read(timeout);
	}

	// px4, handle reading error
	if (!valid) { HandleReadError(); }

	return value;
}

bool RoboClaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
	bool valid;
	uint16_t value = Read2(address, GETENCODERMODE, &valid);

	if (valid) {
		M1mode = value >> 8;
		M2mode = value;
	}

	return valid;
}

bool RoboClaw::SetM1EncoderMode(uint8_t address, uint8_t mode)
{
	return write_n(3, address, SETM1ENCODERMODE, mode);
}

bool RoboClaw::SetM2EncoderMode(uint8_t address, uint8_t mode)
{
	return write_n(3, address, SETM2ENCODERMODE, mode);
}

bool RoboClaw::WriteNVM(uint8_t address)
{
	return write_n(2, address, WRITENVM);
}
