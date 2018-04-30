/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file dynamixel_position.c
 * Dynamixel position driver.
 *
 * @author Zachary Taylor	<ztaylor@ethz.ch>
 *
 * A simple controller for working with dynamixel servos that use protocal 2.0.
 * The servos are set into extended position mode and can be given an angle between -512*PI and 512*PI. 
 * The angle is given via actuator control group 2, unlike most other actuators its value is not normalized.
 * All feedback from the servos is disabled to allow higher throughput when using the ttl servos.
 */

#include <drivers/drv_hrt.h>
#include <math.h>
#include <poll.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <unistd.h>

#include "message.h"

#define DEFAULT_UART "/dev/ttyS1"  /* Serial1 */
#define DEFAULT_BAUDRATE B115200
#define DEFAULT_IGNORE_DISARM true
#define ANGLE_RES 651.898646904f  /* 4096 / (2*pi) */

static int _thread_should_exit = false; /**< Deamon exit flag */
static int _thread_running = false;     /**< Deamon status flag */
static int _deamon_task;                /**< Handle of deamon task / thread */
static const char _daemon_name[] = "dynamixel_position";
static const char _commandline_usage[] =
    "sends values (interpreted as angles in radians) in Control Group #2 to up "
    "to 8 dynamixels\n"
    "usage: dynamixel_position start|status|stop [-d <device> -b <baudrate> -u "
    "<update_rate_hz> -i <ignore_disarm>]";

struct termios _uart_config_original;
int _uart = 0;

/**
 * Sets uart parameters and opens it.
 */
void open_uart(const char *device, const speed_t baudrate);

/**
 * Writes a given message to the uart.
 */
bool write_uart(const uint8_t *msg, int msg_length);

/**
 * Sends the specified angles to the dynamixels.
 */
bool set_angles(const float32 *angles);

/**
 * Sets up and enables torque for the dynamixels.
 */
bool enable(void);

/**
 * Disables torque in the dynamixels.
 */
bool disable(void);

/**
 * Enables/disables the dynamixels, in response to the armed status of the system.
 */
void update_armed_status(int armed_sub, bool ignore_disarm);

/**
 * Sets parameters specified on the command line.
 */
void get_commandline_values(int argc, char *argv[], char **device,
                            speed_t *baudrate, bool *ignore_disarm);

/**
 * Main loop that passes specified angles to the dynamixels.
 */
int dynamixel_position_thread_main(int argc, char *argv[]);

void open_uart(const char *device, const speed_t baudrate) {
  /* open uart */
  _uart = open(device, O_RDWR | O_NOCTTY);

  if (_uart < 0) {
    err(1, "ERR: opening %s", device);
  }

  /* Back up the original uart configuration to restore it after exit */
  int termios_state;

  if ((termios_state = tcgetattr(_uart, &_uart_config_original)) < 0) {
    close(_uart);
    err(1, "ERR: %s: %d", device, termios_state);
  }

  /* Fill the struct for the new configuration */
  struct termios uart_config;
  tcgetattr(_uart, &uart_config);

  /* Clear ONLCR flag (which appends a CR for every LF) */
  uart_config.c_oflag &= ~ONLCR;

  /* Set baud rate */
  if (cfsetispeed(&uart_config, baudrate) < 0 ||
      cfsetospeed(&uart_config, baudrate) < 0) {
    close(_uart);
    err(1, "ERR: %s: %d (cfsetispeed, cfsetospeed)", device, termios_state);
  }

  if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
    close(_uart);
    err(1, "ERR: %s (tcsetattr)", device);
  }
}

bool write_uart(const uint8_t *msg, int msg_length) {
  return write(_uart, msg, msg_length) != 0;
}

bool set_angles(const float32 *angles) {
  int32_t int_angles[NUM_DYNAMIXELS];
  for (uint16_t i = 0; i < NUM_DYNAMIXELS; ++i) {
    int_angles[i] = (int)(((float32)ANGLE_RES) * angles[i]);
  }

  uint8_t msg[WRITE_32_BIT_MESSAGE_SIZE];
  build_write_32_bit_message(GOAL_POSITION_ADDRESS, int_angles, msg);

  if (!write_uart(msg, WRITE_32_BIT_MESSAGE_SIZE)) {
    return false;
  }

  return true;
}

bool enable(void) {
  uint8_t enable_flags[NUM_DYNAMIXELS];
  uint8_t disable_flags[NUM_DYNAMIXELS];
  uint8_t control_flags[NUM_DYNAMIXELS];
  for (uint8_t i = 0; i < NUM_DYNAMIXELS; ++i) {
    enable_flags[i] = 1;
    disable_flags[i] = 0;
    control_flags[i] = EXTENDED_POSITION_CONTROL_MODE;
  }

  uint8_t msg[WRITE_8_BIT_MESSAGE_SIZE];

  /* disable msg status feedback */
  build_write_8_bit_message(RETURN_LEVEL_SET_ADDRESS, disable_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  /* set to extended position control mode */
  build_write_8_bit_message(OPERATING_MODE_ADDRESS, control_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  /* enable servo */
  build_write_8_bit_message(ENABLE_ADDRESS, enable_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  /* turn on led to get some feedback */
  build_write_8_bit_message(LED_ADDRESS, enable_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  return true;
}

bool disable(void) {
  uint8_t disable_flags[NUM_DYNAMIXELS];
  for (uint8_t i = 0; i < NUM_DYNAMIXELS; ++i) {
    disable_flags[i] = 0;
  }

  uint8_t msg[WRITE_8_BIT_MESSAGE_SIZE];

  /* disable servo */
  build_write_8_bit_message(ENABLE_ADDRESS, disable_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  /* turn off led to get some feedback */
  build_write_8_bit_message(LED_ADDRESS, disable_flags, msg);
  if (!write_uart(msg, WRITE_8_BIT_MESSAGE_SIZE)) {
    return false;
  }

  return true;
}

void update_armed_status(int armed_sub, bool ignore_disarm) {
  bool armed_updated;
  orb_check(armed_sub, &armed_updated);

  if (armed_updated) {
    struct actuator_armed_s armed_status;
    orb_copy(ORB_ID(actuator_armed), armed_sub, &armed_status);

    if (armed_status.lockdown || armed_status.force_failsafe ||
        armed_status.in_esc_calibration_mode ||
        (!armed_status.armed && !ignore_disarm)) {
      if (!disable()) {
        warn("failed to disable dynamixels");
      }
    } else {
      if (!enable()) {
        warn("failed to enable dynamixels");
      }
    }
  }
}

void get_commandline_values(int argc, char *argv[], char **device,
                            speed_t *baudrate, bool *ignore_disarm) {
  *device = DEFAULT_UART;
  *baudrate = DEFAULT_BAUDRATE;
  *ignore_disarm = DEFAULT_IGNORE_DISARM;

  for (int i = 0; i < argc && argv[i]; i++) {
    if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
      if (argc > i + 1) {
        *device = argv[i + 1];
      } else {
        _thread_running = false;
        errx(1, "missing parameter to -d\n%s", _commandline_usage);
      }
    }
    if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baudrate") == 0) {
      if (argc > i + 1) {
        if (strcmp(argv[i + 1], "57600") == 0) {
          *baudrate = B57600;
        } else if (strcmp(argv[i + 1], "115200") == 0) {
          *baudrate = B115200;
        } else {
          _thread_running = false;
          errx(1,
               "invalid baudrate set, currently only 57600 and 115200 are "
               "supported");
        }
      } else {
        _thread_running = false;
        errx(1, "missing parameter to -b\n%s", _commandline_usage);
      }
    }
    if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--ignore_disarm") == 0) {
      if (argc > i + 1) {
        if (strcmp(argv[i + 1], "true") == 0) {
          *ignore_disarm = true;
        } else if (strcmp(argv[i + 1], "false") == 0) {
          *ignore_disarm = false;
        } else {
          _thread_running = false;
          errx(
              1,
              "invalid parameter to -i, possible values are 'true' or 'false'");
        }
      } else {
        _thread_running = false;
        errx(1, "missing parameter to -i\n%s", _commandline_usage);
      }
    }
  }
}

int dynamixel_position_thread_main(int argc, char *argv[]) {
  _thread_running = true;

  char *device;
  speed_t baudrate;
  bool ignore_disarm;

  get_commandline_values(argc, argv, &device, &baudrate, &ignore_disarm);

  open_uart(device, baudrate);

  if (_uart == 0) {
    return PX4_ERROR;
  }

  int sub_fd = orb_subscribe(ORB_ID(actuator_controls_2));
  int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

  /* limit the update rate to something the uart can easily handle */
  if (baudrate == B57600) {
    orb_set_interval(sub_fd, 20);
  } else {
    orb_set_interval(sub_fd, 10);
  }

  px4_pollfd_struct_t fds[] = {
      {.fd = sub_fd, .events = POLLIN},
  };

  while (!_thread_should_exit) {
    int poll_ret = px4_poll(fds, 1, 1000);

    if ((poll_ret > 0) && fds[0].revents & POLLIN) {
      update_armed_status(armed_sub, ignore_disarm);

      struct actuator_controls_s angles_in_struct;
      orb_copy(ORB_ID(actuator_controls_2), sub_fd, &angles_in_struct);
      if (!set_angles(angles_in_struct.control)) {
        warn("failed to set dynamixel angles!");
      }
    }
  }

  close(_uart);

  _thread_running = false;

  return PX4_OK;
}

__EXPORT int dynamixel_position_main(int argc, char *argv[]);

/**
 * Process command line arguments and start the daemon.
 */
int dynamixel_position_main(int argc, char *argv[]) {
  if (argc < 1) {
    errx(1, "missing command\n%s", _commandline_usage);
  }

  if (!strcmp(argv[1], "start")) {
    if (_thread_running) {
      warnx("already running");
      exit(0);
    }

    _thread_should_exit = false;
    _deamon_task = px4_task_spawn_cmd(
        _daemon_name, SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1500,
        dynamixel_position_thread_main,
        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop")) {
    _thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status")) {
    if (_thread_running) {
      warnx("is running");
    } else {
      warnx("not started");
    }

    exit(0);
  }

  errx(1, "unrecognized command\n%s", _commandline_usage);
}
