/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Ivan Ovinnikov <oivan@ethz.ch>
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
 * @file fixedwing_control.c
 * Implementation of a fixed wing attitude and position controller.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <math.h>
#include <termios.h>
#include <time.h>
#include <arch/board/up_hrt.h>
#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <nuttx/spi.h>
#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/param/param.h>

__EXPORT int fixedwing_control_main(int argc, char *argv[]);

#define PID_DT 0.01f
#define PID_SCALER 0.1f
#define PID_DERIVMODE_CALC 0
#define HIL_MODE 32
#define AUTO -1000
#define MANUAL 3000
#define SERVO_MIN 1000
#define SERVO_MAX 2000

pthread_t control_thread;
pthread_t nav_thread;
pthread_t servo_thread;

/**
 * Servo channels function enumerator used for
 * the servo writing part
 */
enum SERVO_CHANNELS_FUNCTION {

	AIL_1    = 0,
	AIL_2    = 1,
	MOT      = 2,
	ACT_1    = 3,
	ACT_2    = 4,
	ACT_3    = 5,
	ACT_4    = 6,
	ACT_5    = 7
};

/**
 * The plane_data structure.
 *
 * The plane data structure is the local storage of all the flight information of the aircraft
 */
typedef struct {
	double lat;
	double lon;
	float alt;
	float vx;
	float vy;
	float vz;
	float yaw;
	float hdg;
	float pitch;
	float roll;
	float yawspeed;
	float pitchspeed;
	float rollspeed;
	float rollb;	/* body frame angles */
	float pitchb;
	float yawb;
	float p;
	float q;
	float r;	/* body angular rates */

	/* PID parameters*/

	float Kp_att;
	float Ki_att;
	float Kd_att;
	float Kp_pos;
	float Ki_pos;
	float Kd_pos;
	float intmax_att;
	float intmax_pos;

	/* Next waypoint*/

	float wp_x;
	float wp_y;
	float wp_z;

	/* Setpoints */

	float airspeed;
	float groundspeed;
	float roll_setpoint;
	float pitch_setpoint;
	float throttle_setpoint;

	/* Navigation mode*/
	int mode;

} plane_data_t;

/**
 * The control_outputs structure.
 *
 * The control outputs structure contains the control outputs
 * of the aircraft
 */
typedef struct {
	float roll_ailerons;
	float pitch_elevator;
	float yaw_rudder;
	float throttle;
	// set the aux values to 0 per default
	float aux1;
	float aux2;
	float aux3;
	float aux4;
	uint8_t mode;	// HIL_ENABLED: 32
	uint8_t nav_mode;
} control_outputs_t;

/**
 * Generic PID algorithm with PD scaling
 */
static float pid(float error, float error_deriv, uint16_t dt, float scaler, float K_p, float K_i, float K_d, float intmax);

/*
 * Output calculations
 */

static void calc_body_angular_rates(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
static void calc_rotation_matrix(float roll, float pitch, float yaw, float x, float y, float z);
static void calc_bodyframe_angles(float roll, float pitch, float yaw);
static float calc_bearing(void);
static float calc_roll_ail(void);
static float calc_pitch_elev(void);
static float calc_yaw_rudder(float hdg);
static float calc_throttle(void);
static float calc_gnd_speed(void);
static void get_parameters(plane_data_t *plane_data);
static float calc_roll_setpoint(void);
static float calc_pitch_setpoint(void);
static float calc_throttle_setpoint(void);
static float calc_wp_distance(void);
static void set_plane_mode(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

plane_data_t plane_data;
control_outputs_t control_outputs;
float scaler = 1; //M_PI;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Calculates the PID control output given an error.
 * 
 * Incorporates PD scaling and low-pass filter for the derivative component.
 *
 * @param error the input error
 * @param error_deriv the derivative of the input error
 * @param dt time constant
 * @param scaler PD scaler
 * @param K_p P gain
 * @param K_i I gain
 * @param K_d D gain
 * @param intmax Integration limit
 *
 * @return the PID control output
 */

static float pid(float error, float error_deriv, uint16_t dt, float scale, float K_p, float K_i, float K_d, float intmax)
{
	float Kp = K_p;
	float Ki = K_i;
	float Kd = K_d;
	float delta_time = dt;
	float lerror;
	float imax = intmax;
	float integrator;
	float derivative;
	float lderiv;
	int fCut = 20;		/* anything above 20 Hz is considered noise - low pass filter for the derivative */
	float output = 0;

	output += error * Kp;

	if ((fabsf(Kd) > 0) && (dt > 0)) {

		if (PID_DERIVMODE_CALC) {
			derivative = (error - lerror) / delta_time;

			/*
			 * discrete low pass filter, cuts out the
			 * high frequency noise that can drive the controller crazy
			 */
			float RC = 1.0f / (2.0f * M_PI_F * fCut);
			derivative = lderiv +
				     (delta_time / (RC + delta_time)) * (derivative - lderiv);

			/* update state */
			lerror 	= error;
			lderiv  = derivative;

		} else {
			derivative = error_deriv;
		}

		/* add in derivative component */
		output 	+= Kd * derivative;
	}

	//printf("PID derivative %i\n", (int)(1000*derivative));

	/* scale the P and D components with the PD scaler */
	output *= scale;

	/* Compute integral component if time has elapsed */
	if ((fabsf(Ki) > 0) && (dt > 0)) {
		integrator 		+= (error * Ki) * scaler * delta_time;

		if (integrator < -imax) {
			integrator = -imax;

		} else if (integrator > imax) {
			integrator = imax;
		}

		output += integrator;
	}

	//printf("PID Integrator %i\n", (int)(1000*integrator));

	return output;
}

PARAM_DEFINE_FLOAT(FW_ATT_P, 2.0f);
PARAM_DEFINE_FLOAT(FW_ATT_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_ATT_D, 0.0f);
PARAM_DEFINE_FLOAT(FW_ATT_AWU, 5.0f);
PARAM_DEFINE_FLOAT(FW_ATT_LIM, 10.0f);

PARAM_DEFINE_FLOAT(FW_POS_P, 2.0f);
PARAM_DEFINE_FLOAT(FW_POS_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_POS_D, 0.0f);
PARAM_DEFINE_FLOAT(FW_POS_AWU, 5.0f);
PARAM_DEFINE_FLOAT(FW_POS_LIM, 10.0f);

/**
 * Load parameters from global storage.
 *
 * @param plane_data Fixed wing data structure
 *
 * Fetches the current parameters from the global parameter storage and writes them
 * to the plane_data structure
 */
static void get_parameters(plane_data_t * pdata)
{
	static bool initialized = false;
	static param_t att_p;
	static param_t att_i;
	static param_t att_d;
	static param_t att_awu;
	static param_t att_lim;

	static param_t pos_p;
	static param_t pos_i;
	static param_t pos_d;
	static param_t pos_awu;
	static param_t pos_lim;

	if (!initialized) {
		att_p = param_find("FW_ATT_P");
		att_i = param_find("FW_ATT_I");
		att_d = param_find("FW_ATT_D");
		att_awu = param_find("FW_ATT_AWU");
		att_lim = param_find("FW_ATT_LIM");

		pos_p = param_find("FW_POS_P");
		pos_i = param_find("FW_POS_I");
		pos_d = param_find("FW_POS_D");
		pos_awu = param_find("FW_POS_AWU");
		pos_lim = param_find("FW_POS_LIM");

		initialized = true;
	}

	param_get(att_p, &(pdata->Kp_att));
	param_get(att_i, &(pdata->Ki_att));
	param_get(att_d, &(pdata->Kd_att));
	param_get(pos_p, &(pdata->Kp_pos));
	param_get(pos_i, &(pdata->Ki_pos));
	param_get(pos_d, &(pdata->Kd_pos));
	param_get(att_awu, &(pdata->intmax_att));
	param_get(pos_awu, &(pdata->intmax_pos));
	pdata->airspeed = 10;
	pdata->wp_x =  48;
	pdata->wp_y =  8;
	pdata->wp_z =  100;
	pdata->mode = 1;
}

/**
 * Calculates the body angular rates.
 *
 * Calculates the rates of the plane using inertia matrix and
 * writes them to the plane_data structure
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param rollspeed
 * @param pitchspeed
 * @param yawspeed
 *
 */
static void calc_body_angular_rates(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	plane_data.p = rollspeed - sinf(pitch) * yawspeed;
	plane_data.q = cosf(roll) * pitchspeed + sinf(roll) * cosf(pitch) * yawspeed;
	plane_data.r = -sinf(roll) * pitchspeed + cosf(roll) * cosf(pitch) * yawspeed;
}

/**
 *
 * Calculates the attitude angles in the body reference frame.
 *
 * Writes them to the plane data structure
 *
 * @param roll
 * @param pitch
 * @param yaw
 */

static void calc_bodyframe_angles(float roll, float pitch, float yaw)
{
	plane_data.rollb = cosf(yaw) * cosf(pitch) * roll +
			   (cosf(yaw) * sinf(pitch) * sinf(roll) + sinf(yaw) * cosf(roll)) * pitch
			   + (-cosf(yaw) * sinf(pitch) * cosf(roll)  + sinf(yaw) * sinf(roll)) * yaw;
	plane_data.pitchb = -sinf(yaw) * cosf(pitch) * roll +
			    (-sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll)) * pitch
			    + (sinf(yaw) * sinf(pitch) * cosf(roll) + cosf(yaw) * sinf(roll)) * yaw;
	plane_data.yawb = sinf(pitch) * roll - cosf(pitch) * sinf(roll) * pitch + cosf(pitch) * cosf(roll) * yaw;
}

/**
 * calc_rotation_matrix
 *
 * Calculates the rotation matrix
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param x
 * @param y
 * @param z
 *
 */

static void calc_rotation_matrix(float roll, float pitch, float yaw, float x, float y, float z)
{
	plane_data.rollb = cosf(yaw) * cosf(pitch) * x +
			   (cosf(yaw) * sinf(pitch) * sinf(roll) + sinf(yaw) * cosf(roll)) * y
			   + (-cosf(yaw) * sinf(pitch) * cosf(roll)  + sinf(yaw) * sinf(roll)) * z;
	plane_data.pitchb = -sinf(yaw) * cosf(pitch) * x +
			    (-sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll)) * y
			    + (sinf(yaw) * sinf(pitch) * cosf(roll) + cosf(yaw) * sinf(roll)) * z;
	plane_data.yawb = sinf(pitch) * x - cosf(pitch) * sinf(roll) * y + cosf(pitch) * cosf(roll) * z;
}

/**
 * calc_bearing
 *
 * Calculates the bearing error of the plane compared to the waypoints
 *
 * @return bearing Bearing error
 *
 */
static float calc_bearing()
{
	float bearing = M_PI_F/2.0f + (float)atan2(-(plane_data.wp_y - plane_data.lat), (plane_data.wp_x - plane_data.lon));

	if (bearing < 0.0f) {
		bearing += 2*M_PI_F;
	}

	return bearing;
}

/**
 * calc_roll_ail
 *
 * Calculates the roll ailerons control output
 *
 * @return Roll ailerons control output (-1,1)
 */

static float calc_roll_ail()
{
	float ret = pid((plane_data.roll_setpoint - plane_data.roll), plane_data.rollspeed, PID_DT, PID_SCALER,
			plane_data.Kp_att, plane_data.Ki_att, plane_data.Kd_att, plane_data.intmax_att);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_pitch_elev
 *
 * Calculates the pitch elevators control output
 *
 * @return Pitch elevators control output (-1,1)
 */
static float calc_pitch_elev()
{
	float ret = pid((plane_data.pitch_setpoint - plane_data.pitch), plane_data.pitchspeed, PID_DT, PID_SCALER,
			plane_data.Kp_att, plane_data.Ki_att, plane_data.Kd_att, plane_data.intmax_att);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_yaw_rudder
 *
 * Calculates the yaw rudder control output (only if yaw rudder exists on the model)
 *
 * @return Yaw rudder control output (-1,1)
 *
 */
static float calc_yaw_rudder(float hdg)
{
	float ret = pid((plane_data.yaw - abs(hdg)), plane_data.yawspeed, PID_DT, PID_SCALER,
			plane_data.Kp_pos, plane_data.Ki_pos, plane_data.Kd_pos, plane_data.intmax_pos);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_throttle
 *
 * Calculates the throttle control output
 *
 * @return Throttle control output (0,1)
 */

static float calc_throttle()
{
	float ret = pid(plane_data.throttle_setpoint - calc_gnd_speed(), 0, PID_DT, PID_SCALER,
			plane_data.Kp_pos, plane_data.Ki_pos, plane_data.Kd_pos, plane_data.intmax_pos);

	if (ret < 0.2f)
		return 0.2f;

	if (ret > 1.0f)
		return 1.0f;

	return ret;
}

/**
 * calc_gnd_speed
 *
 * Calculates the ground speed using the x and y components
 *
 * Input: none (operation on global data)
 *
 * Output: Ground speed of the plane
 *
 */

static float calc_gnd_speed()
{
	float gnd_speed = sqrtf(plane_data.vx * plane_data.vx + plane_data.vy * plane_data.vy);
	return gnd_speed;
}

/**
 * calc_wp_distance
 *
 * Calculates the distance to the next waypoint
 *
 * @return the distance to the next waypoint
 *
 */

static float calc_wp_distance()
{
	return sqrtf((plane_data.lat - plane_data.wp_y) * (plane_data.lat - plane_data.wp_y) +
		     (plane_data.lon - plane_data.wp_x) * (plane_data.lon - plane_data.wp_x));
}

/**
 * calc_roll_setpoint
 *
 * Calculates the offset angle for the roll plane,
 * saturates at +- 35 deg.
 *
 * @return setpoint on which attitude control should stabilize while changing heading
 *
 */

static float calc_roll_setpoint()
{
	float setpoint = 0;

	setpoint = calc_bearing() - plane_data.yaw;

	if (setpoint < (-35.0f/180.0f)*M_PI_F)
		return (-35.0f/180.0f)*M_PI_F;

	if (setpoint > (35/180.0f)*M_PI_F)
		return (-35.0f/180.0f)*M_PI_F;

	return setpoint;
}

/**
 * calc_pitch_setpoint
 *
 * Calculates the offset angle for the pitch plane
 * saturates at +- 35 deg.
 *
 * @return setpoint on which attitude control should stabilize while changing altitude
 *
 */

static float calc_pitch_setpoint()
{
	float setpoint = 0.0f;

	// if (plane_data.mode == TAKEOFF) {
	// 	setpoint = ((35/180.0f)*M_PI_F);

	// } else {
		setpoint = atanf((plane_data.wp_z - plane_data.alt) / calc_wp_distance());

		if (setpoint < (-35.0f/180.0f)*M_PI_F)
			return (-35.0f/180.0f)*M_PI_F;

		if (setpoint > (35/180.0f)*M_PI_F)
			return (-35.0f/180.0f)*M_PI_F;
	// }

	return setpoint;
}

/**
 * calc_throttle_setpoint
 *
 * Calculates the throttle setpoint for different flight modes
 *
 * @return throttle output setpoint
 *
 */

static float calc_throttle_setpoint()
{
	float setpoint = 0;

	// // if TAKEOFF full throttle
	// if (plane_data.mode == TAKEOFF) {
	// 	setpoint = 60.0f;
	// }

	// // if CRUISE - parameter value
	// if (plane_data.mode == CRUISE) {
		setpoint = plane_data.airspeed;
	// }

	// // if LAND no throttle
	// if (plane_data.mode == LAND) {
	// 	setpoint = 0.0f;
	// }

	return setpoint;
}

/*
 * fixedwing_control_main
 *
 * @param argc number of arguments
 * @param argv argument array
 *
 * @return 0
 *
 */

int fixedwing_control_main(int argc, char *argv[])
{
	/* default values for arguments */
	char *fixedwing_uart_name = "/dev/ttyS1";
	char *commandline_usage = "\tusage: %s -d fixedwing-devicename\n";

	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				fixedwing_uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0]);
				return 0;
			}
		}
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[fixedwing control] started\n");

	/* output structs */
	struct actuator_controls_s actuators;
	struct actuator_armed_s armed;
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	armed.armed = false;
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* Subscribe to global position, attitude and rc */
	struct vehicle_global_position_s global_pos;
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_setpoint_s global_setpoint;
	int global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	/* declare and safely initialize all structs */
	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* Mainloop setup */
	unsigned int loopcounter = 0;
	unsigned int failcounter = 0;

	/* Control constants */
	control_outputs.mode = HIL_MODE;
	control_outputs.nav_mode = 0;

	/* Servo setup */

	/*
	 * Main control, navigation and servo routine
	 */

	while(1) {
		/*
		 * DATA Handling
		 * Fetch current flight data
		 */

		/* get position, attitude and rc inputs */
		// XXX add error checking
		orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
		orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);
		
		/* get a local copy of system state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of attitude setpoint */
		orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);

		/* scaling factors are defined by the data from the APM Planner
		 * TODO: ifdef for other parameters (HIL/Real world switch)
		 */

		/* position values*/
		plane_data.lat = global_pos.lat / 10000000.0;
		plane_data.lon = global_pos.lon / 10000000.0;
		plane_data.alt = global_pos.alt / 1000.0f;
		plane_data.vx = global_pos.vx / 100.0f;
		plane_data.vy = global_pos.vy / 100.0f;
		plane_data.vz = global_pos.vz / 100.0f;

		/* attitude values*/
		plane_data.roll = att.roll;
		plane_data.pitch = att.pitch;
		plane_data.yaw = att.yaw;
		plane_data.rollspeed = att.rollspeed;
		plane_data.pitchspeed = att.pitchspeed;
		plane_data.yawspeed = att.yawspeed;

		/* parameter values */
		if (loopcounter % 20 == 0) {
			get_parameters(&plane_data);
		}

		/* Attitude control part */

		if (verbose && loopcounter % 20 == 0) {
			/******************************** DEBUG OUTPUT ************************************************************/

			printf("Parameter: %i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n", (int)plane_data.Kp_att, (int)plane_data.Ki_att,
					(int)plane_data.Kd_att, (int)plane_data.intmax_att, (int)plane_data.Kp_pos, (int)plane_data.Ki_pos,
					(int)plane_data.Kd_pos, (int)plane_data.intmax_pos, (int)plane_data.airspeed,
					(int)plane_data.wp_x, (int)plane_data.wp_y, (int)plane_data.wp_z);

			printf("PITCH SETPOINT: %i\n", (int)(100.0f*plane_data.pitch_setpoint));
			printf("ROLL SETPOINT: %i\n", (int)(100.0f*plane_data.roll_setpoint));
			printf("THROTTLE SETPOINT: %i\n", (int)(100.0f*calc_throttle_setpoint()));

	//		printf("\n\nVx: %i\t Vy: %i\t Current speed:%i\n\n", (int)plane_data.vx, (int)plane_data.vy, (int)(calc_gnd_speed()));

	//		printf("Current Altitude: %i\n\n", (int)plane_data.alt);

			printf("\nAttitude values: \n R:%i \n P: %i \n Y: %i \n\n RS: %i \n PS: %i \n YS: %i \n ",
					(int)(180.0f * plane_data.roll/M_PI_F), (int)(180.0f * plane_data.pitch/M_PI_F), (int)(180.0f * plane_data.yaw/M_PI_F),
					(int)(180.0f * plane_data.rollspeed/M_PI_F), (int)(180.0f * plane_data.pitchspeed/M_PI_F), (int)(180.0f * plane_data.yawspeed)/M_PI_F);

	//		printf("\nBody Rates: \n P: %i \n Q: %i \n R: %i \n",
	//				(int)(10000 * plane_data.p), (int)(10000 * plane_data.q), (int)(10000 * plane_data.r));

			printf("\nCalculated outputs: \n R: %8.4f\n P: %8.4f\n Y: %8.4f\n T: %8.4f \n",
					att_sp.roll_body, att_sp.pitch_body,
					att_sp.yaw_body, att_sp.thrust);

			/************************************************************************************************************/
		}

		/* Calculate the P,Q,R body rates of the aircraft */
		//calc_body_angular_rates(plane_data.roll, plane_data.pitch, plane_data.yaw,
		//		plane_data.rollspeed, plane_data.pitchspeed, plane_data.yawspeed);

		/* Calculate the body frame angles of the aircraft */
		//calc_bodyframe_angles(plane_data.roll,plane_data.pitch,plane_data.yaw);

		// if (manual.override_mode_switch < XXX) {


		/* Navigation part */

		// Get Waypoint

		// XXX FIXME

		//else if (manual.override_mode_switch > MANUAL) {	// AUTO mode
		
		/* calculate setpoint based on current position and waypoint */
		att_sp.roll_body = calc_roll_setpoint();
		att_sp.pitch_body = calc_pitch_setpoint();
		att_sp.thrust = calc_throttle_setpoint();

		/* calculate the control outputs based on roll / pitch / yaw setpoints */
		actuators.control[0] = calc_roll_ail();
		actuators.control[1] = calc_pitch_elev();
		actuators.control[2] = calc_yaw_rudder(att.yaw);
		actuators.control[3] = calc_throttle();

		/* publish attitude setpoint (for MAVLink) */
		orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

		/* publish actuator setpoints (for mixer) */

		/* arming state depends on commander arming state */
		armed.armed = state.flag_system_armed;
		orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		loopcounter++;

		/* 20Hz loop*/
		usleep(50000);
	}

	return 0;
}
