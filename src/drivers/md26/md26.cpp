/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file md25.cpp
 *
 * Driver for MD25 I2C Motor Driver
 *
 * references:
 * http://www.robot-electronics.co.uk/htm/md25tech.htm
 * http://www.robot-electronics.co.uk/files/rpi_md25.c
 *
 */
#include <px4_config.h>
#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
//#include <getopt.h>
#include <lib/conversion/rotation.h>

#include <arch/board/board.h>
#include <mavlink/mavlink_log.h>
#include <uORB/topics/actuator_controls_0.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>



/**
 * Deamon management function.
 */
extern "C" __EXPORT int md26_main(int argc, char *argv[]);

// registers
enum {
	// RW: read/write
	// R: read
	REG_SPEED1_RW = 0,
	REG_SPEED2_RW,
	REG_ENC1A_R,
	REG_ENC1B_R,
	REG_ENC1C_R,
	REG_ENC1D_R,
	REG_ENC2A_R,
	REG_ENC2B_R,
	REG_ENC2C_R,
	REG_ENC2D_R,
	REG_BATTERY_VOLTS_R,
	REG_MOTOR1_CURRENT_R,
	REG_MOTOR2_CURRENT_R,
	REG_SW_VERSION_R,
	REG_ACCEL_RATE_RW,
	REG_MODE_RW,
	REG_COMMAND_RW,
};

/**
 * This is a driver for the MD25 motor controller utilizing the I2C interface.
 */
class MD25 : public device::I2C
{
public:

    /**
    * modes
    *
    * NOTE: this driver assumes we are always
    * in mode 0!
    *
    * seprate speed mode:
    *  motor speed1 = speed1
    *  motor speed2 = speed2
    * turn speed mode:
    *   motor speed1 = speed1 + speed2
    *  motor speed2 = speed2 - speed2
    * unsigned:
    *  full rev (0), stop(128), full fwd (255)
    * signed:
    *  full rev (-127), stop(0), full fwd (128)
    *
    * modes numbers:
    * 0 : unsigned separate (default)
    * 1 : signed separate
    * 2 : unsigned turn
    * 3 : signed turn
    */
    enum e_mode {
        MODE_UNSIGNED_SPEED = 0,
        MODE_SIGNED_SPEED,
        MODE_UNSIGNED_SPEED_TURN,
        MODE_SIGNED_SPEED_TURN,
    };

    /** commands */
    enum e_cmd {
        CMD_RESET_ENCODERS = 32,
        CMD_DISABLE_SPEED_REGULATION = 48,
        CMD_ENABLE_SPEED_REGULATION = 49,
        CMD_DISABLE_TIMEOUT = 50,
        CMD_ENABLE_TIMEOUT = 51,
        CMD_CHANGE_I2C_SEQ_0 = 160,
        CMD_CHANGE_I2C_SEQ_1 = 170,
        CMD_CHANGE_I2C_SEQ_2 = 165,
    };

    /** control channels */
    enum e_channels {
        CH_SPEED_LEFT = 0,
        CH_SPEED_RIGHT
    };

    /**
     * constructor
     * @param deviceName the name of the device e.g. "/dev/md25"
     * @param bus the I2C bus
     * @param address the adddress on the I2C bus
     * @param speed the speed of the I2C communication
     */
    MD25(const char *deviceName,
         int bus,
         uint16_t address,
         uint32_t speed = 100000);

    /**
     * deconstructor
     */
    virtual ~MD25();

    /**
     * @return software version
     */
    uint8_t getVersion();

    /**
     * @return speed of motor, normalized (-1, 1)
     */
    float getMotor1Speed();

    /**
     * @return speed of motor 2, normalized (-1, 1)
     */
    float getMotor2Speed();

    /**
     * @return number of rotations since reset
     */
    float getRevolutions1();

    /**
     * @return number of rotations since reset
     */
    float getRevolutions2();

    /**
     * @return battery voltage, volts
     */
    float getBatteryVolts();

    /**
     * @return motor 1 current, amps
     */
    float getMotor1Current();

    /**
     * @return motor 2 current, amps
     */
    float getMotor2Current();

    /**
     * @return the motor acceleration
     * controls motor speed change (1-10)
     * accel rate  | time for full fwd. to full rev.
     *  1          | 6.375 s
     *  2          | 1.6 s
     *  3          | 0.675 s
     *  5(default) | 1.275 s
     * 10          | 0.65 s
     */
    uint8_t getMotorAccel();

    /**
     * @return motor output mode
     * */
    e_mode getMode();

    /**
     * @return current command register value
     */
    e_cmd getCommand();

    /**
     * resets the encoders
     * @return non-zero -> error
     * */
    int resetEncoders();

    /**
     * enable/disable speed regulation
     * @return non-zero -> error
     */
    int setSpeedRegulation(bool enable);

    /**
     * set the timeout for the motors
     * enable/disable timeout (motor stop)
     * after 2 sec of no i2c messages
     * @return non-zero -> error
     */
    int setTimeout(bool enable);

    /**
     * sets the device address
     * can only be done with one MD25
     * on the bus
     * @return non-zero -> error
     */
    int setDeviceAddress(uint8_t address);

    /**
     * set motor acceleration
     * @param accel
     * controls motor speed change (1-10)
     * accel rate  | time for full fwd. to full rev.
     *  1          | 6.375 s
     *  2          | 1.6 s
     *  3          | 0.675 s
     *  5(default) | 1.275 s
     * 10          | 0.65 s
     */
    int setMotorAccel(uint8_t accel);

    /**
     * set motor 1 speed
     * @param normSpeed normalize speed between -1 and 1
     * @return non-zero -> error
    */
    int setMotor1Speed(float normSpeed);

    /**
     * set motor 2 speed
     * @param normSpeed normalize speed between -1 and 1
     * @return non-zero -> error
     */
    int setMotor2Speed(float normSpeed);

    /**
     * main update loop that updates MD25 motor
     * speeds based on actuator publication
     */
    void update();
    int Init();
    int start();
    //int     start();
    static void task_main_trampoline(int argc, char *argv[]);

    /**
     * probe for device
     */
    virtual int probe();

    /**
     * search for device
     */
    int search();

    /**
     * read data from i2c
     */
    int readData();

    /**
     * print status
     */
    void status(char *string, size_t n);
    int       _task;
private:
    /** poll structure for control packets */
    struct pollfd _controlPoll;

    /** actuator controls subscription */
        struct actuator_controls_s _actuators;
        orb_id_t    _control_topics;
        //uORB::Subscription<actuator_controls_s> _actuators;
        int         _control_subs;


    // local copy of data from i2c device
    uint8_t _version;
    float _motor1Speed;
    float _motor2Speed;
    float _revolutions1;
    float _revolutions2;
    float _batteryVoltage;
    float _motor1Current;
    float _motor2Current;
    uint8_t _motorAccel;
    e_mode _mode;
    e_cmd _command;
     bool   _task_should_exit;
    // private methods
    int _writeUint8(uint8_t reg, uint8_t value);
    int _writeInt8(uint8_t reg, int8_t value);
    float _uint8ToNorm(uint8_t value);
    uint8_t _normToUint8(float value);

    /**
     * set motor control mode,
     * this driver assumed we are always in mode 0
     * so we don't let the user change the mode
     * @return non-zero -> error
     */
    int _setMode(e_mode);
};

// unit testing

// sine testing
int md25Sine(const char *deviceName, uint8_t bus, uint8_t address, float amplitude, float frequency);

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78

MD25::MD25(const char *deviceName, int bus,
       uint16_t address, uint32_t speed) :
        I2C("MD25", deviceName, bus,address
        #ifdef __PX4_NUTTX
                , 100000
        #endif
               ),
    //_controlPoll{},
    //_actuators(NULL, ORB_ID(actuator_controls_0), 20),
    _task(-1),
    _version(0),
    _motor1Speed(0),
    _motor2Speed(0),
    _revolutions1(0),
    _revolutions2(0),
    _batteryVoltage(0),
    _motor1Current(0),
    _motor2Current(0),
    _motorAccel(0),
    _mode(MODE_UNSIGNED_SPEED),
    _command(CMD_RESET_ENCODERS),
    _task_should_exit(false)
{

}
namespace
{

MD25  *md25_control;

int Test( uint8_t bus, uint8_t address);

int Test( uint8_t bus, uint8_t address)
{
    // setup

    md25_control = new MD25("/dev/md25",bus,address);

    if (OK != md25_control->Init()) {
                           delete md25_control;
                           md25_control = nullptr;
                           warnx("init failed");
                           return 1;
                       }

    // print status
    char buf[400];
    md25_control->status(buf, sizeof(buf));
    //printf("%s\n", buf);

    // setup for test
    md25_control->setSpeedRegulation(false);
    md25_control->setTimeout(true);
    float dt = 0.1;
    float speed = 0.2;
    float t = 0;

    // motor 1 test
    printf("md25 test: spinning motor 1 forward for 1 rev at 0.1 speed\n");
    t = 0;

    while (true) {
        t += dt;
        md25_control->setMotor1Speed(speed);
        md25_control->readData();
        usleep(1000000 * dt);

        if (md25_control->getRevolutions1() > 1) {
            printf("finished 1 revolution fwd\n");
            break;
        }

        if (t > 2.0f) { break; }
    }

    md25_control->setMotor1Speed(0);
    printf("revolution of wheel 1: %8.4f\n", double(md25_control->getRevolutions1()));
    md25_control->resetEncoders();

    t = 0;

    while (true) {
        t += dt;
        md25_control->setMotor1Speed(-speed);
        md25_control->readData();
        usleep(1000000 * dt);

        if (md25_control->getRevolutions1() < -1) {
            printf("finished 1 revolution rev\n");
            break;
        }

        if (t > 2.0f) { break; }
    }

    md25_control->setMotor1Speed(0);
    printf("revolution of wheel 1: %8.4f\n", double(md25_control->getRevolutions1()));
    md25_control->resetEncoders();

    // motor 2 test
    printf("md25 test: spinning motor 2 forward for 1 rev at 0.1 speed\n");
    t = 0;

    while (true) {
        t += dt;
        md25_control->setMotor2Speed(speed);
        md25_control->readData();
        usleep(1000000 * dt);

        if (md25_control->getRevolutions2() > 1) {
            printf("finished 1 revolution fwd\n");
            break;
        }

        if (t > 2.0f) { break; }
    }

    md25_control->setMotor2Speed(0);
    printf("revolution of wheel 2: %8.4f\n", double(md25_control->getRevolutions2()));
    md25_control->resetEncoders();

    t = 0;

    while (true) {
        t += dt;
        md25_control->setMotor2Speed(-speed);
        md25_control->readData();
        usleep(1000000 * dt);

        if (md25_control->getRevolutions2() < -1) {
            printf("finished 1 revolution rev\n");
            break;
        }

        if (t > 2.0f) { break; }
    }

    md25_control->setMotor2Speed(0);
    printf("revolution of wheel 2: %8.4f\n", double(md25_control->getRevolutions2()));
    md25_control->resetEncoders();

    printf("Test complete\n");
    if (md25_control != nullptr) {
        delete md25_control;
        md25_control = nullptr;
    }

    return 0;
}
}/*end namespace*/

MD25::~MD25()
{
    if (_task != -1) {
            /* tell the task we want it to go away */
            _task_should_exit = true;

            unsigned i = 10;

            do {
                /* wait 50ms - it should wake every 100ms or so worst-case */
                usleep(50000);

                /* if we have given up, kill it */
                if (--i == 0) {
                    px4_task_delete(_task);
                    break;
                }

            } while (_task != -1);
        }
    md25_control = nullptr;
}

int MD25::Init()
{
        int ret;
        ret = I2C::init();

        if (ret != OK) {
            warnx("I2C init failed");
            return ret;
        }
        // setup default settings, reset encoders
           setMotor1Speed(0);
           setMotor2Speed(0);
           resetEncoders();
           _setMode(MD25::MODE_UNSIGNED_SPEED);
           setSpeedRegulation(false);
           setMotorAccel(10);
           setTimeout(true);

        _control_topics=ORB_ID(actuator_controls_0);
           // setup control polling
        _control_subs = orb_subscribe(_control_topics);
        _controlPoll.fd = _control_subs;
        _controlPoll.events = POLLIN;
        memset(&_actuators, 0, sizeof(_actuators));
        return OK;
}

int MD25::readData()
{
	uint8_t sendBuf[1];
	sendBuf[0] = REG_SPEED1_RW;
	uint8_t recvBuf[17];
	int ret = transfer(sendBuf, sizeof(sendBuf),
			   recvBuf, sizeof(recvBuf));

	if (ret == OK) {
		_version = recvBuf[REG_SW_VERSION_R];
		_motor1Speed = _uint8ToNorm(recvBuf[REG_SPEED1_RW]);
		_motor2Speed = _uint8ToNorm(recvBuf[REG_SPEED2_RW]);
		_revolutions1 = -int32_t((recvBuf[REG_ENC1A_R] << 24) +
					 (recvBuf[REG_ENC1B_R] << 16) +
					 (recvBuf[REG_ENC1C_R] << 8)  +
					 recvBuf[REG_ENC1D_R]) / 360.0;
		_revolutions2 = -int32_t((recvBuf[REG_ENC2A_R] << 24) +
					 (recvBuf[REG_ENC2B_R] << 16) +
					 (recvBuf[REG_ENC2C_R] << 8)  +
					 recvBuf[REG_ENC2D_R]) / 360.0;
		_batteryVoltage = recvBuf[REG_BATTERY_VOLTS_R] / 10.0;
		_motor1Current = recvBuf[REG_MOTOR1_CURRENT_R] / 10.0;
		_motor2Current = recvBuf[REG_MOTOR2_CURRENT_R] / 10.0;
		_motorAccel = recvBuf[REG_ACCEL_RATE_RW];
		_mode = e_mode(recvBuf[REG_MODE_RW]);
		_command = e_cmd(recvBuf[REG_COMMAND_RW]);
	}

	return ret;
}

void MD25::status(char *string, size_t n)
{
	snprintf(string, n,
		 "version:\t%10d\n" \
		 "motor 1 speed:\t%10.2f\n" \
		 "motor 2 speed:\t%10.2f\n" \
		 "revolutions 1:\t%10.2f\n" \
		 "revolutions 2:\t%10.2f\n" \
		 "battery volts :\t%10.2f\n" \
		 "motor 1 current :\t%10.2f\n" \
		 "motor 2 current :\t%10.2f\n" \
		 "motor accel :\t%10d\n" \
		 "mode :\t%10d\n" \
		 "command :\t%10d\n",
		 getVersion(),
		 double(getMotor1Speed()),
		 double(getMotor2Speed()),
		 double(getRevolutions1()),
		 double(getRevolutions2()),
		 double(getBatteryVolts()),
		 double(getMotor1Current()),
		 double(getMotor2Current()),
		 getMotorAccel(),
		 getMode(),
		 getCommand());
}

uint8_t MD25::getVersion()
{
	return _version;
}

float MD25::getMotor1Speed()
{
	return _motor1Speed;
}

float MD25::getMotor2Speed()
{
	return _motor2Speed;
}

float MD25::getRevolutions1()
{
	return _revolutions1;
}

float MD25::getRevolutions2()
{
	return _revolutions2;
}

float MD25::getBatteryVolts()
{
	return _batteryVoltage;
}

float MD25::getMotor1Current()
{
	return _motor1Current;
}
float MD25::getMotor2Current()
{
	return _motor2Current;
}

uint8_t MD25::getMotorAccel()
{
	return _motorAccel;
}

MD25::e_mode MD25::getMode()
{
	return _mode;
}

MD25::e_cmd MD25::getCommand()
{
	return _command;
}

int MD25::resetEncoders()
{
	return _writeUint8(REG_COMMAND_RW,
			   CMD_RESET_ENCODERS);
}

int MD25::_setMode(e_mode mode)
{
	return _writeUint8(REG_MODE_RW,
			   mode);
}

int MD25::setSpeedRegulation(bool enable)
{
	if (enable) {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_ENABLE_SPEED_REGULATION);

	} else {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_DISABLE_SPEED_REGULATION);
	}
}

int MD25::setTimeout(bool enable)
{
	if (enable) {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_ENABLE_TIMEOUT);

	} else {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_DISABLE_TIMEOUT);
	}
}

int MD25::setDeviceAddress(uint8_t address)
{
	uint8_t sendBuf[1];
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_0;
	int ret = OK;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	usleep(5000);
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_1;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	usleep(5000);
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_2;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	return OK;
}

int MD25::setMotorAccel(uint8_t accel)
{
	return _writeUint8(REG_ACCEL_RATE_RW,
			   accel);
}

int MD25::setMotor1Speed(float value)
{
	return _writeUint8(REG_SPEED1_RW,
			   _normToUint8(value));
}

int MD25::setMotor2Speed(float value)
{
	return _writeUint8(REG_SPEED2_RW,
			   _normToUint8(value));
}

void MD25::update()
{
	// wait for an actuator publication,
	// check for exit condition every second
	// note "::poll" is required to distinguish global
	// poll from member function for driver

    bool updated;

    while (!_task_should_exit) {
	//if (::poll(&_controlPoll, 1, 1000) < 0) { return; } // poll error

        orb_check(_control_subs, &updated);
      //  warnx("update motor if event");
        if (updated){
	  //  if (_controlPoll.revents & POLLIN) {
            orb_copy(_control_topics, _control_subs, &_actuators);

            fprintf(stderr, "actuateur1 : %f\n",(double)_actuators.control[0] );
            fprintf(stderr, "actuateur2 : %f\n",(double)_actuators.control[1] );

	        setMotor1Speed(_actuators.control[CH_SPEED_LEFT]);
	        setMotor2Speed(_actuators.control[CH_SPEED_RIGHT]);

	    }
    }
       warnx("exiting");

}

int MD25::probe()
{
	uint8_t goodAddress = 0;
	bool found = false;
	int ret = OK;

	// try initial address first, if good, then done
	if (readData() == OK) { return ret; }

	// try all other addresses
	uint8_t testAddress = 0;

	//printf("searching for MD25 address\n");
	while (true) {
		set_address(testAddress);
		ret = readData();

		if (ret == OK && !found) {
			//printf("device found at address: 0x%X\n", testAddress);
			if (!found) {
				found = true;
				goodAddress = testAddress;
			}
		}

		if (testAddress > 254) {
			break;
		}

		testAddress++;
	}

	if (found) {
		set_address(goodAddress);
		return OK;

	} else {
		set_address(0);
		return ret;
	}
}

int MD25::search()
{
	uint8_t goodAddress = 0;
	bool found = false;
	int ret = OK;
	// try all other addresses
	uint8_t testAddress = 0;

	//printf("searching for MD25 address\n");
	while (true) {
		set_address(testAddress);
		ret = readData();

		if (ret == OK && !found) {
			printf("device found at address: 0x%X\n", testAddress);

			if (!found) {
				found = true;
				goodAddress = testAddress;
			}
		}

		if (testAddress > 254) {
			break;
		}

		testAddress++;
	}

	if (found) {
		set_address(goodAddress);
		return OK;

	} else {
		set_address(0);
		return ret;
	}
}

int MD25::_writeUint8(uint8_t reg, uint8_t value)
{
	uint8_t sendBuf[2];
	sendBuf[0] = reg;
	sendBuf[1] = value;
	return transfer(sendBuf, sizeof(sendBuf),
			nullptr, 0);
}

int MD25::_writeInt8(uint8_t reg, int8_t value)
{
	uint8_t sendBuf[2];
	sendBuf[0] = reg;
	sendBuf[1] = value;
	return transfer(sendBuf, sizeof(sendBuf),
			nullptr, 0);
}

float MD25::_uint8ToNorm(uint8_t value)
{
	// TODO, should go from 0 to 255
	// possibly should handle this differently
	return (value - 128) / 127.0;
}

uint8_t MD25::_normToUint8(float value)
{
	if (value > 1) { value = 1; }

	if (value < -1) { value = -1; }

	// TODO, should go from 0 to 255
	// possibly should handle this differently
	return 127 * value + 128;
}





void MD25::task_main_trampoline(int argc, char *argv[])
{

       md25_control->update();

}


int md25Sine(const char *deviceName, uint8_t bus, uint8_t address, float amplitude, float frequency)
{
	printf("md25 sine: starting\n");

	// setup
	MD25 md25("/dev/md25", bus, address);

	// print status
	char buf[400];
	md25.status(buf, sizeof(buf));
	printf("%s\n", buf);

	// setup for test
	md25.setSpeedRegulation(false);
	md25.setTimeout(true);
	float dt = 0.01;
	float t_final = 60.0;
	//float prev_revolution = md25.getRevolutions1();

	// debug publication


	// sine wave for motor 1
	md25.resetEncoders();

	while (true) {

		// input
		uint64_t timestamp = hrt_absolute_time();
		float t = timestamp / 1000000.0f;

		float input_value = amplitude * sinf(2 *float(M_PI) * frequency * t);
		md25.setMotor1Speed(input_value);

		// output
		md25.readData();
		//float current_revolution = md25.getRevolutions1();



		if (t > t_final) { break; }

		// update for next step
		//prev_revolution = current_revolution;

		// sleep
		usleep(1000000 * dt);
	}

	md25.setMotor1Speed(0);

	printf("md25 sine complete\n");
	return 0;
}




/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */

int md26_main(int argc, char *argv[])
{
  if (argc < 2) {
           warn("missing command");
       }

    if (!strcmp(argv[1], "start")) {
               const char *deviceName = "/dev/md26";

               uint8_t bus = strtoul(argv[2], nullptr, 0);

               uint8_t address = strtoul(argv[3], nullptr, 0);
               if (md25_control == nullptr)
                   md25_control = new MD25(deviceName, bus, address);

               if (md25_control == nullptr) {
                   warnx("alloc failed");
                   exit(1);
               }
               if (OK != md25_control->Init()) {
                    delete md25_control;
                    md25_control = nullptr;
                    warnx("init failed");
                    exit(1);
                 }
               if ((md25_control->_task == -1) && (OK != md25_control->start())) {
                   delete md25_control;
                   md25_control = nullptr;
                   warnx("start failed");
                   exit(1);
               }

               exit(0);
           }

    if (!strcmp(argv[1], "test")) {

            if (argc < 4) {
                warn("usage: md25 test bus address\n");
                exit(0);
            }

            uint8_t bus = strtoul(argv[2], nullptr, 0);

            uint8_t address = strtoul(argv[3], nullptr, 0);

            Test(bus,address);

            exit(0);
        }

    if (!strcmp(argv[1], "stop")) {
        delete md25_control;
        md25_control = nullptr;
       }

   warnx("unrecognized command");
   exit(1);

}

int MD25::start()
{
    warnx("[MD25] starting\n");

    ASSERT(_task == -1);

    /* start the task */


    _task = px4_task_spawn_cmd("md26",
                             SCHED_DEFAULT,
                             SCHED_PRIORITY_MAX - 10,
                             2048,
                             (px4_main_t)&MD25::task_main_trampoline,
                              nullptr);


    if (_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
