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
 * @file iqinetics_underactuated_propeller.c
 * Minimal application example for an underactuated propeller vehicle powered by IQinetics motors
 *
 * @author Matthew Piccoli <matt@iqinetics.com>
 */

/**
 * To use TELEM2 as motor control port, set SYS_COMPANION param = 0
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <termios.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include "generic_interface.hpp"
// #include "virtual_swashplate_client.hpp"
#include "voltage_super_position_client.hpp"
#include "propeller_motor_control_client.hpp"

extern "C" __EXPORT int iqinetics_underactuated_propeller_main(int argc, char *argv[]);
static volatile bool thread_should_exit = false;   /**< Daemon exit flag */
static volatile bool thread_running = false;   /**< Daemon status flag */
static volatile int daemon_task;       /**< Handle of daemon task / thread */

int iqinetics_underactuated_propeller_thread_main(int argc, char *argv[]);
int send_msgs_to_uart(GenericInterface& com, int serial_fd);
int setup_uart(char *uart_name, int &serial_fd);

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int iqinetics_underactuated_propeller_main(int argc, char *argv[])
{
  if (argc < 2) {
    PX4_ERR("missing command");
    exit(1);
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      PX4_INFO("already running\n");
      /* this is not an error */
      exit(0);
    }

    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawning");

    thread_should_exit = false;
    daemon_task = px4_task_spawn_cmd("iqinetics_underactuated_propeller",
             SCHED_DEFAULT,
             SCHED_PRIORITY_DEFAULT - 1,
             5000,
             iqinetics_underactuated_propeller_thread_main,
             (argv) ? (char *const *)&argv[2] : (char *const *)NULL);

    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawned");
    exit(0);
  }

  if (!strcmp(argv[1], "stop")) {
    PX4_INFO("stopping");
    thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      PX4_INFO("running");

    } else {
      PX4_INFO("stopped");
    }

    exit(0);
  }

  PX4_ERR("unrecognized command");
  exit(1);
}

int iqinetics_underactuated_propeller_thread_main(int argc, char *argv[])
{
	PX4_INFO("IQinetics Underactuated Propeller Thread Loading");

	// Check input arguments
	if (argc < 3) {
	  PX4_ERR("need 2 serial port names as arguments");
	  exit(1);
  }

	// Start UART
  char *uart_name1 = argv[1];
  char *uart_name2 = argv[2];

  int serial_fds[2] = {-1, -1};

  if(setup_uart(uart_name1, serial_fds[0]) == 0)
    PX4_INFO("Opened %s with fd %d", uart_name1, serial_fds[0]);
  else
    exit(1);
  if(setup_uart(uart_name2, serial_fds[1]) == 0)
    PX4_INFO("Opened %s with fd %d", uart_name2, serial_fds[1]);
  else
    exit(1);

  // load params
  param_t max_speed_param;
  param_t max_pulse_volts_param;
  param_t max_yaw_param;
  float max_speed_value = 0.0f;
  float max_pulse_volts_value = 0.0f;
  float max_yaw_value = 0.0f;

  max_speed_param = param_find("PROP_MAX_SPEED");
  max_pulse_volts_param = param_find("PROP_MAX_PULSE");
  max_yaw_param = param_find("PROP_MAX_YAW");

  if (max_speed_param != PARAM_INVALID) {
    param_get(max_speed_param, &max_speed_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_SPEED param invalid");
  }

  if (max_pulse_volts_param != PARAM_INVALID) {
    param_get(max_pulse_volts_param, &max_pulse_volts_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_PULSE param invalid");
  }

  if (max_yaw_param != PARAM_INVALID) {
    param_get(max_yaw_param, &max_yaw_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_YAW param invalid");
  }

	// Make a communication interface object
  GenericInterface com1;
  GenericInterface com2;
	// Make a objects that talk to the module
	// VirtualSwashplateClient     swash1(0);
  // VirtualSwashplateClient     swash2(0);
  PropellerMotorControlClient pmc1(0);
  PropellerMotorControlClient pmc2(0);
  VoltageSuperPositionClient  vsc1(0); 
  VoltageSuperPositionClient  vsc2(0);

	// subscribe to actuator_controls_0 topic
	int actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	// subscribe to actuator_armed topic
	int actuator_arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	/* limit the update rate to 1 kHz */
	orb_set_interval(actuator_ctrl_sub_fd, 1);
	orb_set_interval(actuator_arm_sub_fd, 1);

	px4_pollfd_struct_t fds[2];
	fds[0].fd = actuator_ctrl_sub_fd;
	fds[0].events = POLLIN;
	fds[1].fd = actuator_arm_sub_fd;
	fds[1].events = POLLIN;

	// initialize variables
	int error_counter = 0;
	thread_running = true;
	bool is_armed = false;

	// main while loop for this thread
	while(!thread_should_exit)
	{
		/* wait for sensor update of 2 file descriptors for 10 ms (100hz) */
		int poll_ret = px4_poll(fds, 2, 10);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a 10ms");
		}
		else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0)
			{
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		}
		else
		{
		  if (fds[1].revents & POLLIN)
		  {
		    // Get the actuator armed data
        struct actuator_armed_s actuator_arm_raw;
        orb_copy(ORB_ID(actuator_armed), actuator_arm_sub_fd, &actuator_arm_raw);

        // Remember armed state
        is_armed = actuator_arm_raw.armed;
		  }

			if (is_armed && (fds[0].revents & POLLIN))
			{
				// Get the actuator control data
				struct actuator_controls_s actuator_raw;
				orb_copy(ORB_ID(actuator_controls_0), actuator_ctrl_sub_fd, &actuator_raw);

				// --------------------------------------------------------------------
				// Vehicle behavior goes here

				// swash1.speed_.set(com1,actuator_raw.control[3]*max_speed_value + actuator_raw.control[2]*max_yaw_value); // INDEX_THROTTLE = 3, INDEX_YAW = 2
				// swash1.roll_.set(com1,actuator_raw.control[0]*max_pulse_volts_value); // INDEX_ROLL = 0
				// swash1.pitch_.set(com1,actuator_raw.control[1]*max_pulse_volts_value); // INDEX_PITCH = 1
        // swash2.speed_.set(com2,actuator_raw.control[3]*max_speed_value - actuator_raw.control[2]*max_yaw_value); // INDEX_THROTTLE = 3, INDEX_YAW = 2
        // swash2.roll_.set(com2,actuator_raw.control[0]*max_pulse_volts_value); // INDEX_ROLL = 0
        // swash2.pitch_.set(com2,actuator_raw.control[1]*max_pulse_volts_value); // INDEX_PITCH = 1

        float x_roll    = actuator_raw.control[0]*max_pulse_volts_value;
        float y_pitch   = actuator_raw.control[1]*max_pulse_volts_value; 
        float amplitude = sqrt(pow(x_roll,2.0f)+pow(y_pitch,2.0f));

        pmc1.ctrl_velocity_.set(com1,actuator_raw.control[3]*max_speed_value + actuator_raw.control[2]*max_yaw_value); // INDEX_THROTTLE = 3, INDEX_YAW = 2
        vsc1.phase_.set(com1,atan2(x_roll,y_pitch));
        vsc1.amplitude_.set(com1,amplitude);

        pmc2.ctrl_velocity_.set(com2,actuator_raw.control[3]*max_speed_value - actuator_raw.control[2]*max_yaw_value); // INDEX_THROTTLE = 3, INDEX_YAW = 2
        vsc2.phase_.set(com1,atan2(x_roll,y_pitch));
        vsc2.amplitude_.set(com1,amplitude);

        int send_ret = send_msgs_to_uart(com1, serial_fds[0]);
        if(send_ret != 0)
          PX4_WARN("serial1 send error %d", send_ret);

        send_ret = send_msgs_to_uart(com2, serial_fds[1]);
        if(send_ret != 0)
          PX4_WARN("serial2 send error %d", send_ret);
        // End vehicle behavior
        // --------------------------------------------------------------------
			}

		}
	}
	PX4_INFO("exiting");
  thread_running = false;

  fflush(stdout);
	return 0;
}

int send_msgs_to_uart(GenericInterface& com, int serial_fd)
{
  // This buffer is for passing around messages.
  uint8_t communication_buffer[256];
  // Stores length of message to send or receive
  uint8_t communication_length;

  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(serial_fd >= 0 && com.GetTxBytes(communication_buffer,communication_length))
  {
    //TODO::do the write in a while loop, decrementing com_length by written and incrementing buffer address
    uint8_t written = ::write(serial_fd, communication_buffer, communication_length);
    ::fsync(serial_fd);
    if(written != communication_length)
      return -1;
    return 0;
  }
  return -2;
}

/**
 * setup_uart initializes a uart port to 115200 8N1
 * uart_name is the port string descriptor (/dev/ttySx)
 * serial_fd is the resulting file descriptor for the port
 * returns 0 if successful, -1 if setup is unable to setup the port
 */
int setup_uart(char *uart_name, int &serial_fd)
{
  PX4_INFO("opening port %s", uart_name);

  serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

  unsigned speed = 115200;

  if (serial_fd < 0) {
    PX4_WARN("failed to open port: %s", uart_name);
    return -1;
  }

  /* Try to set baud rate */
  struct termios uart_config;
  int termios_state;

  /* Back up the original uart configuration to restore it after exit */
  if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
    PX4_WARN("ERR GET CONF %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  /* Set baud rate */
  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
    PX4_WARN("ERR SET BAUD %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
    PX4_WARN("ERR SET CONF %s\n", uart_name);
    close(serial_fd);
    return -1;
  }
  return 0;
}
