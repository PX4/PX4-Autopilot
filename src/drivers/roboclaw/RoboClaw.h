#pragma once

// modified version of code from orion robotics
// added functions for compatibility with px4 - James Goppert

#include <stdarg.h>

// px4 added
#include <inttypes.h> // uint16_t
#include <poll.h> // strtoul

/******************************************************************************
* Definitions
******************************************************************************/

#define _RC_VERSION 10 // software version of this library
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class RoboClaw
{
	// px4 added
	int uart;
	struct pollfd uartPoll[1];

	uint32_t timeout;
	bool ack;	//ack mode only supported on 3.1.8 and newer firmware

	enum {M1FORWARD = 0,
	      M1BACKWARD = 1,
	      SETMINMB = 2,
	      SETMAXMB = 3,
	      M2FORWARD = 4,
	      M2BACKWARD = 5,
	      M17BIT = 6,
	      M27BIT = 7,
	      MIXEDFORWARD = 8,
	      MIXEDBACKWARD = 9,
	      MIXEDRIGHT = 10,
	      MIXEDLEFT = 11,
	      MIXEDFB = 12,
	      MIXEDLR = 13,
	      GETM1ENC = 16,
	      GETM2ENC = 17,
	      GETM1SPEED = 18,
	      GETM2SPEED = 19,
	      RESETENC = 20,
	      GETVERSION = 21,
	      GETMBATT = 24,
	      GETLBATT = 25,
	      SETMINLB = 26,
	      SETMAXLB = 27,
	      SETM1PID = 28,
	      SETM2PID = 29,
	      GETM1ISPEED = 30,
	      GETM2ISPEED = 31,
	      M1DUTY = 32,
	      M2DUTY = 33,
	      MIXEDDUTY = 34,
	      M1SPEED = 35,
	      M2SPEED = 36,
	      MIXEDSPEED = 37,
	      M1SPEEDACCEL = 38,
	      M2SPEEDACCEL = 39,
	      MIXEDSPEEDACCEL = 40,
	      M1SPEEDDIST = 41,
	      M2SPEEDDIST = 42,
	      MIXEDSPEEDDIST = 43,
	      M1SPEEDACCELDIST = 44,
	      M2SPEEDACCELDIST = 45,
	      MIXEDSPEEDACCELDIST = 46,
	      GETBUFFERS = 47,
	      GETCURRENTS = 49,
	      MIXEDSPEED2ACCEL = 50,
	      MIXEDSPEED2ACCELDIST = 51,
	      M1DUTYACCEL = 52,
	      M2DUTYACCEL = 53,
	      MIXEDDUTYACCEL = 54,
	      READM1PID = 55,
	      READM2PID = 56,
	      SETMAINVOLTAGES = 57,
	      SETLOGICVOLTAGES = 58,
	      GETMINMAXMAINVOLTAGES = 59,
	      GETMINMAXLOGICVOLTAGES = 60,
	      SETM1POSPID = 61,
	      SETM2POSPID = 62,
	      READM1POSPID = 63,
	      READM2POSPID = 64,
	      M1SPEEDACCELDECCELPOS = 65,
	      M2SPEEDACCELDECCELPOS = 66,
	      MIXEDSPEEDACCELDECCELPOS = 67,
	      GETTEMP = 82,
	      GETERROR = 90,
	      GETENCODERMODE = 91,
	      SETM1ENCODERMODE = 92,
	      SETM2ENCODERMODE = 93,
	      WRITENVM = 94
	     };
public:

	// public methods
	RoboClaw(const char *port, uint32_t tout, bool doack = false);	//ack option only available on 3.1.8 and newer firmware

	virtual ~RoboClaw();

	bool ForwardM1(uint8_t address, uint8_t speed);
	bool BackwardM1(uint8_t address, uint8_t speed);
	bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage);
	bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage);
	bool ForwardM2(uint8_t address, uint8_t speed);
	bool BackwardM2(uint8_t address, uint8_t speed);
	bool ForwardBackwardM1(uint8_t address, uint8_t speed);
	bool ForwardBackwardM2(uint8_t address, uint8_t speed);
	bool ForwardMixed(uint8_t address, uint8_t speed);
	bool BackwardMixed(uint8_t address, uint8_t speed);
	bool TurnRightMixed(uint8_t address, uint8_t speed);
	bool TurnLeftMixed(uint8_t address, uint8_t speed);
	bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
	bool LeftRightMixed(uint8_t address, uint8_t speed);
	uint32_t ReadEncM1(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadEncM2(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadSpeedM1(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadSpeedM2(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	bool ResetEncoders(uint8_t address);
	bool ReadVersion(uint8_t address, char *version);
	uint16_t ReadMainBatteryVoltage(uint8_t address, bool *valid = NULL);
	uint16_t ReadLogicBattVoltage(uint8_t address, bool *valid = NULL);
	bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);
	bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);
	bool SetM1VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps);
	bool SetM2VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps);
	uint32_t ReadISpeedM1(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadISpeedM2(uint8_t address, uint8_t *status = NULL, bool *valid = NULL);
	bool DutyM1(uint8_t address, uint16_t duty);
	bool DutyM2(uint8_t address, uint16_t duty);
	bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
	bool SpeedM1(uint8_t address, uint32_t speed);
	bool SpeedM2(uint8_t address, uint32_t speed);
	bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
	bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
	bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2,
			       uint8_t flag = 0);
	bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2,
				    uint32_t distance2, uint8_t flag = 0);
	bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
	bool ReadCurrents(uint8_t address, uint8_t &current1, uint8_t &current2);
	bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
	bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2,
				      uint32_t speed2, uint32_t distance2, uint8_t flag = 0);
	bool DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel);
	bool DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel);
	// px4 changes, changed duty1/ duty2 to int16_t from uint16_t
	bool DutyAccelM1M2(uint8_t address, int16_t duty1, uint16_t accel1, int16_t duty2, uint16_t accel2);
	bool ReadM1VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps);
	bool ReadM2VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps);
	bool SetMainVoltages(uint8_t address, uint16_t min, uint16_t max);
	bool SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max);
	bool ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max);
	bool ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t &max);
	bool SetM1PositionPID(uint8_t address, float kd, float kp, float ki, float kiMax, uint32_t deadzone, uint32_t min,
			      uint32_t max);
	bool SetM2PositionPID(uint8_t address, float kd, float kp, float ki, float kiMax, uint32_t deadzone, uint32_t min,
			      uint32_t max);
	bool ReadM1PositionPID(uint8_t address, float &Kp, float &Ki, float &Kd, float &KiMax, uint32_t &DeadZone,
			       uint32_t &Min, uint32_t &Max);
	bool ReadM2PositionPID(uint8_t address, float &Kp, float &Ki, float &Kd, float &KiMax, uint32_t &DeadZone,
			       uint32_t &Min, uint32_t &Max);
	bool SpeedAccelDeccelPositionM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position,
					uint8_t flag);
	bool SpeedAccelDeccelPositionM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position,
					uint8_t flag);
	bool SpeedAccelDeccelPositionM1M2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t deccel1,
					  uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag);
	bool ReadTemp(uint8_t address, uint16_t &temp);
	uint8_t ReadError(uint8_t address, bool *valid = NULL);
	bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
	bool SetM1EncoderMode(uint8_t address, uint8_t mode);
	bool SetM2EncoderMode(uint8_t address, uint8_t mode);
	bool WriteNVM(uint8_t address);

private:
	// px4 added
	bool write(uint8_t byte);
	uint8_t read(uint32_t tout);
	void HandleReadError();

	bool write_n(uint8_t byte, ...);
	bool read_n(uint8_t byte, uint8_t address, uint8_t cmd, ...);
	uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid);
	uint16_t Read2(uint8_t address, uint8_t cmd, bool *valid);

};
