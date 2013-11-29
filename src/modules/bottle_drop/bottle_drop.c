/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file bottle_drop.c
 * bottle_drop application
 * 
 * @author Dominik Juchli <juchlid@ethz.ch>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>


#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>

PARAM_DEFINE_FLOAT(BD_HEIGHT, 60.0f);
PARAM_DEFINE_FLOAT(BD_GPROPERTIES, 0.03f);
PARAM_DEFINE_FLOAT(BD_TURNRADIUS, 70.0f);
PARAM_DEFINE_FLOAT(BD_PRECISION, 1.0f);
PARAM_DEFINE_INT32(BD_APPROVAL, 0);




static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

static bool drop = false;

/**
 * daemon management function.
 */
__EXPORT int bottle_drop_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int bottle_drop_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int bottle_drop_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("bottle_drop",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 bottle_drop_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "drop")) {
		drop = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int bottle_drop_thread_main(int argc, char *argv[]) {

	warnx("starting\n");

	bool updated = false;

	float height;		// height at which the normal should be dropped NED
	float z_0;		// ground properties
	float turn_radius;	// turn radius of the UAV
	float precision;	// Expected precision of the UAV
	bool drop_approval;	// if approval is given = true, otherwise = false

	thread_running = true;

	/* XXX TODO: create, publish and read in wind speed topic */
	struct wind_speed_s {
		float vx; // m/s
		float vy; // m/s
		float altitude; // m
	} wind_speed;

	wind_speed.vx = 4.2f;
	wind_speed.vy = 0.0f;
	wind_speed.altitude = 62.0f;


	/* XXX TODO: create, publish and read in target position in NED*/
	struct position_s {
		double lat; //degrees 1E7
		double lon; //degrees 1E7
		float alt; //m
	} target_position, drop_position, flight_vector_s, flight_vector_e;

	target_position.lat =  47.385806;
	target_position.lon =  8.589093;
	target_position.alt = 0.0f;


	// constant
	float g = 9.81f;               		// constant of gravity [m/s^2]
	float m = 0.5f;                		// mass of bottle [kg]
	float rho = 1.2f;              		// air density [kg/m^3]
	float A = (powf(0.063f, 2.0f)/4.0f*M_PI_F);     // Bottle cross section [m^2]
	float dt = 0.01f;             		// step size [s]
	float dt2 = 0.05f;			// step size 2 [s]

	// Has to be estimated by experiment
	float cd = 0.86f;              	// Drag coefficient for a cylinder with a d/l ratio of 1/3 []
	float t_signal = 0.084f;	// Time span between sending the signal and the bottle top reaching level height with the bottom of the plane [s]
	float t_door = 0.7f;		// The time the system needs to open the door + safety, is also the time the palyload needs to safely escape the shaft [s]


	// Definition
	float h_0;						// height over target
	float az;                 				// acceleration in z direction[m/s^2]
	float vz; 						// velocity in z direction [m/s]
	float z; 						// fallen distance [m]
	float h; 						// height over target [m]
	float ax;						// acceleration in x direction [m/s^2]
	float vx;						// ground speed in x direction [m/s]
	float x;					        // traveled distance in x direction [m]
	float vw;                 				// wind speed [m/s]
	float vrx;						// relative velocity in x direction [m/s]
	float v;						// relative speed vector [m/s]
	float Fd;						// Drag force [N]
	float Fdx;						// Drag force in x direction [N]
	float Fdz;						// Drag force in z direction [N]
	float vr; 						// absolute wind speed [m/s]
	float x_drop, y_drop;					// coordinates of the drop point in reference to the target (projection of NED)
	float x_t,y_t;						// coordinates of the target in reference to the target x_t = 0, y_t = 0 (projection of NED)
	float x_l,y_l;						// local position in projected coordinates
	float x_f,y_f;						// to-be position of the UAV after dt2 seconds in projected coordinates
	double x_f_NED, y_f_NED;				// to-be position of the UAV after dt2 seconds in NED
	float distance_open_door;				// The distance the UAV travels during its doors open [m]
	float distance_real = 0;				// The distance between the UAVs position and the drop point [m]
	float future_distance = 0;				// The distance between the UAVs to-be position and the drop point [m]

	// states
	bool state_door = false;				// Doors are closed = false, open = true
	bool state_drop = false;				// Drop occurred = true, Drop din't occur = false
	bool state_run = false;					// A drop was attempted = true, the drop is still in progress = false


	param_t param_height = param_find("BD_HEIGHT");
	param_t param_gproperties = param_find("BD_GPROPERTIES");
	param_t param_turn_radius = param_find("BD_TURNRADIUS");
	param_t param_precision = param_find("BD_PRECISION");
	param_t param_approval = param_find("BD_APPROVAL");


	param_get(param_approval, &drop_approval);
	param_get(param_precision, &precision);
	param_get(param_turn_radius, &turn_radius);
	param_get(param_height, &height);
	param_get(param_gproperties, &z_0);


	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(vehicle_attitude_sub, 100);

	struct vehicle_global_position_s globalpos;
	memset(&globalpos, 0, sizeof(globalpos));
	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	struct parameter_update_s update;
	memset(&update, 0, sizeof(update));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));
	orb_advert_t actuator_pub = orb_advertise(ORB_ID(actuator_controls_1), &actuators);

	struct pollfd fds[] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN }
	};


	while (!thread_should_exit) {

//		warnx("in while!\n");
		// values from -1 to 1

		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			warnx("poll error");

		} else if (ret > 0) {

			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

			orb_check(vehicle_global_position_sub, &updated);
			if (updated){
				/* copy global position */
				orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &globalpos);
			}
			//////////////////////////////////////////////////////////////////// DEBUGGING
			globalpos.lat = 47.384486;
			globalpos.lon =  8.588239;
			globalpos.vx = 18.0f;
			globalpos.vy = 0.0f;
			globalpos.alt = 60.0f;
			globalpos.yaw = M_PI_F/2.0f;


			orb_check(parameter_update_sub, &updated);
			if (updated){
				/* copy global position */
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				/* update all parameters */
			param_get(param_height, &height);
			param_get(param_gproperties, &z_0);
			param_get(param_turn_radius, &turn_radius);
			param_get(param_approval, &drop_approval);
			param_get(param_precision, &precision);


			}

			// Initialization
			 az = g;                 				// acceleration in z direction[m/s^2]
			 vz = 0; 						// velocity in z direction [m/s]
			 z = 0; 						// fallen distance [m]
			 h_0 = globalpos.alt - target_position.alt; 		// height over target at start[m]
			 h = h_0;						// height over target [m]
			 ax = 0;						// acceleration in x direction [m/s^2]
			 vx = globalpos.vx;					// ground speed in x direction [m/s]
			 x = 0;							// traveled distance in x direction [m]
			 vw = 0;                 				// wind speed [m/s]
			 vrx = 0;						// relative velocity in x direction [m/s]
			 v = globalpos.vx;					// relative speed vector [m/s]
			 Fd = 0;						// Drag force [N]
			 Fdx = 0;						// Drag force in x direction [N]
			 Fdz = 0;						// Drag force in z direction [N]
			 vr = sqrt(pow(wind_speed.vx,2) + pow(wind_speed.vy,2)); // absolute wind speed [m/s]
			 distance_open_door = t_door * globalpos.vx;


			 //warnx("absolut wind speed = %.4f", vr); //////////////////////////////////////////////////////////////////// DEBUGGING


			 //warnx("Initialization complete\n"); //////////////////////////////////////////////////////////////////// DEBUGGING


			if (drop_approval && !state_drop)
			{
				//warnx("approval given\n"); //////////////////////////////////////////////////////////////////// DEBUGGING
				// drop here
				//open_now = true;
				//drop = false;
				//drop_start = hrt_absolute_time();

				unsigned counter = 0;

				// Compute the distance the bottle will travel after it is dropped in body frame coordinates --> x
				while( h > 0.05f)
				{
					// z-direction
					vz = vz + az*dt;
					z = z + vz*dt;
					h = h_0 - z;

					// x-direction
					vw = vr*logf(h/z_0)/logf(wind_speed.altitude/z_0);
					vx = vx + ax*dt;
					x = x + vx*dt;
					vrx = vx + vw;

					//Drag force
					v = sqrtf(powf(vz,2.0f) + powf(vrx,2.0f));
					Fd = 0.5f*rho*A*cd*powf(v,2.0f);
					Fdx = Fd*vrx/v;
					Fdz = Fd*vz/v;

					//acceleration
					az = g - Fdz/m;
					ax = -Fdx/m;

				}
				// Compute Drop point
				x = globalpos.vx*t_signal + x;
				map_projection_init(target_position.lat, target_position.lon);
				//warnx("x = %.4f", x); //////////////////////////////////////////////////////////////////// DEBUGGING



				map_projection_project(target_position.lat, target_position.lon, &x_t, &y_t);
				if( vr < 0.001f)		// if there is no wind, an arbitrarily direction is chosen
				{
					vr = 1;
					wind_speed.vx = 1;
					wind_speed.vy = 0;
				}
				x_drop = x_t + x*wind_speed.vx/vr;
				y_drop = y_t + x*wind_speed.vy/vr;
				map_projection_reproject(x_drop, y_drop, &drop_position.lat, &drop_position.lon);
				drop_position.alt = height;
				//warnx("drop point: lat = %.7f , lon = %.7f", drop_position.lat, drop_position.lon); //////////////////////////////////////////////////////////////////// DEBUGGING



				// Compute flight vector
				map_projection_reproject(x_drop + 2*turn_radius*wind_speed.vx/vr, y_drop + 2*turn_radius*wind_speed.vy/vr, &flight_vector_s.lat, &flight_vector_s.lon);
				flight_vector_s.alt = height;
				map_projection_reproject(x_drop - turn_radius*wind_speed.vx/vr, y_drop - turn_radius*wind_speed.vy/vr, &flight_vector_e.lat, &flight_vector_e.lon);
				flight_vector_e.alt = height;
				//warnx("Flight vector: starting point = %.7f  %.7f , end point = %.7f  %.7f", flight_vector_s.lat,flight_vector_s.lon,flight_vector_e.lat,flight_vector_e.lon); //////////////////////////////////////////////////////////////////// DEBUGGING


				// Drop Cancellation if terms are not met
				distance_real = get_distance_to_next_waypoint(globalpos.lat, globalpos.lon, drop_position.lat, drop_position.lon);
				map_projection_project(globalpos.lat, globalpos.lon, &x_l, &y_l);
				x_f = x_l + globalpos.vx*cosf(globalpos.yaw)*dt2 - globalpos.vy*sinf(globalpos.yaw)*dt2; // Attention to sign, has to be checked
				y_f = y_l + globalpos.vy*cosf(globalpos.yaw)*dt2 - globalpos.vx*sinf(globalpos.yaw)*dt2;
				map_projection_reproject(x_f, y_f, &x_f_NED, &y_f_NED);
				future_distance = get_distance_to_next_waypoint(x_f_NED, y_f_NED, drop_position.lat, drop_position.lon);
				//warnx("position to-be: = %.7f  %.7f" ,x_f_NED, y_f_NED ); //////////////////////////////////////////////////////////////////// DEBUGGING

			}

			if(distance_real < distance_open_door && drop_approval)
			{
				actuators.control[0] = -1.0f;	// open door
				actuators.control[1] = 1.0f;
				state_door = true;
			}
			else
			{		// closed door and locked survival kit
				actuators.control[0] = 0.5f;
				actuators.control[1] = -0.5f;
				actuators.control[2] = -0.5f;
				state_door = false;
			}
			if(distance_real < precision && distance_real < future_distance && state_door)  // Drop only if the distance between drop point and actual position is getting larger again
			{
				if(fabsf(acosf(cosf(globalpos.yaw))+acosf(wind_speed.vx)) < 10.0f/180.0f*M_PI_F) // if flight trajectory deviates more than 10 degrees from calculated path, it will no drop
				{
					actuators.control[2] = 0.5f;
					state_drop = true;
					state_run = true;
				}
				else
				{
					state_run = true;
				}
			}


			actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_controls_1), actuator_pub, &actuators);

		}
	}

	warnx("exiting.\n");

	thread_running = false;


	return 0;
}
