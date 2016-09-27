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
 * @file av_estimator_main.cpp
 * Filter main loop, calls both filter classes and retreives data from sensors.
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */


 

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>

#include <mc_quat_vel_control/mc_quat_vel_control_params.h>

#include <mavlink/mavlink_log_handler.h>
#include <drivers/drv_hrt.h>
#include <px4_eigen.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/topics/rc_channels.h>

#include <uORB/topics/wind_estimate.h>

#include "../include/av_estimator_b.h"
#include "../include/av_estimator_a.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "../include/av_estimator_params.h"
#ifdef __cplusplus
}
#endif

extern "C" __EXPORT int av_estimator_main(int argc, char *argv[]);

using namespace Eigen;

#include "../include/av_estimator_main.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int av_estimator_task;			   /**< Handle of deamon task / thread */
int mavlink_fd;

orb_advert_t vel_meas_pub = nullptr;
orb_advert_t wind_vel_pub = nullptr;      //This is nolonger required
float zbar = 0.0f, vzbar = 0.0f, beta_az = 0.0f;
uint64_t baro_prev_time;

int 		_v_rc_sub; 
struct 		rc_channels_s	_v_rc_channels;
int vel_calib_count = 0;
float xvelcalib = 0.0f, yvelcalib = 0.0f, xoff = 0.0f, yoff = 0.0f; 

/* Drag coefficient */
Vector3f Cbar = Vector3f::Zero();


static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: av_estimator {start|stop|status}\n");
	exit(1);
}

void 	vehicle_rc_poll()
{
	bool updated;

	/* Check RC state if vehicle status has changed */
	orb_check(_v_rc_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(rc_channels), _v_rc_sub, &_v_rc_channels);
	}
}

/**
 * The av_estimator app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int av_estimator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		av_estimator_task = px4_task_spawn_cmd("av_estimator",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX - 5,
					      6000,
					      av_estimator_thread_main,
					      (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		while (thread_running){
			usleep(200000);
		}
		
		warnx("stopped");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");
			exit(0);

		} else {
			warnx("not started");
			exit(1);
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


int av_estimator_thread_main(int argc, char *argv[])
{
	/* Daemon state */
	thread_running = true;

	/* Open mavlink log device for error and info messages */
	//mavlink_fd = open(MAVLINK_LOG_DEVICE, 0); MBangura enable

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sub_raw, 3);

	/* Init struct for sensor messages */
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

	/* subscribe to esc data */
	int sub_esc_status = orb_subscribe(ORB_ID(esc_status));
	orb_set_interval(sub_esc_status,50);

	/* Init esc dat structure */
	struct esc_status_s esc_status_raw;
	memset(&esc_status_raw, 0, sizeof(esc_status_raw));

	/* Subscribe to vehicle status */
	int sub_vehicle_status = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(sub_vehicle_status,50);

	/* Init vehicle status data structure */
	struct vehicle_status_s vehicle_status_raw;
	memset(&vehicle_status_raw, 0, sizeof(vehicle_status_raw));	

	/* subscribe to vehicle_gps_position topic MOSES: this is not needed*/
	int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
	orb_set_interval(gps_sub_fd, 50);

	/* gps data struct */
	struct vehicle_gps_position_s rawGPS;
        memset(&rawGPS, 0, sizeof(rawGPS));

        /* subscribe to vehicle_vicon_position topic */
	int vicon_sub_fd = orb_subscribe(ORB_ID(vehicle_vicon_position));
	orb_set_interval(vicon_sub_fd, 50);

	/* vicon data struct */
	struct vehicle_vicon_position_s rawVicon;
        memset(&rawVicon, 0, sizeof(rawVicon));

	/* subscribe to vehicle measured velocity topic */
	int velocity_measured_fd = orb_subscribe(ORB_ID(vehicle_velocity_meas_inertial));
	orb_set_interval(velocity_measured_fd, 50);

	/* Initialize measured velocity data struct */
	struct vehicle_velocity_meas_inertial_s rawMeasuredVelocity;
    memset(&rawMeasuredVelocity, 0, sizeof(rawMeasuredVelocity));

    /* Initialize wind velocity data struct */
	struct wind_estimate_s windVelocity;
    memset(&windVelocity, 0, sizeof(windVelocity));

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	struct av_estimator_params attitude_params;
	struct av_estimator_param_handles attitude_param_handles;

	struct vehicle_velocity_meas_est_body_s meas_body_vel;

	/* initialize parameter handles */
	parameters_init(&attitude_param_handles);
	parameters_update(&attitude_param_handles, &attitude_params);

	/* Offset variables */
	Vector3f gyro_offsets 	= Vector3f::Zero();
	Vector3f acc_offsets 	= Vector3f::Zero();
	Vector3f mu_init 	= Vector3f::Zero();
	int offset_count 	= 0;
	float baro_offset = 0.0f;

	/* Measurement data variables */
	Vector3f omega 	= Vector3f::Zero();
	Vector3f a 	= Vector3f::Zero();
	Vector3f mu 	= Vector3f::Zero();
	Vector3f vbar 	= Vector3f::Zero();

	/* Timestep */
	float dt = 0.005f;
	uint64_t prev_timestamp = 0;		

	/* Barometer outpput data vz z*/ 
	Vector2f baro_data = Vector2f::Zero();
	float baro_alt = 0.0f;

	/* Magnetometer current calibration variables */
	float current_k[12]	= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	float current_c[3]	= {0.0f,0.0f,0.0f};
	float current[4] 	= {0.0f,0.0f,0.0f,0.0f};

	bool initialized = false;
	uint64_t start_time = hrt_absolute_time();
	uint64_t armed_start_time = 0l;

	/* Filter object */
	av_estimator_b filter_b;
	av_estimator_a filter_a;

	
	float prev_rc_chan5;

	memset(&_v_rc_channels, 0, sizeof(_v_rc_channels));

	_v_rc_sub 		= orb_subscribe(ORB_ID(rc_channels));
	vehicle_rc_poll();

	rawMeasuredVelocity.inertial_valid = false;
	prev_rc_chan5 = _v_rc_channels.channels[5];

	/* Main loop*/
	while (!thread_should_exit) {

		int pollingInterval = 50;
		struct pollfd fds[7];
		fds[0].fd = sub_raw;
		fds[0].events = POLLIN;
		fds[1].fd = sub_params;
		fds[1].events = POLLIN;
		fds[2].fd = sub_esc_status;
		fds[2].events = POLLIN;
		fds[3].fd = sub_vehicle_status;
		fds[3].events = POLLIN;
		fds[4].fd = gps_sub_fd;
		fds[4].events = POLLIN;
		fds[5].fd = velocity_measured_fd;
		fds[5].events = POLLIN;
		fds[6].fd = vicon_sub_fd;
		fds[6].events = POLLIN;
		int ret = poll(fds, 7, pollingInterval);


		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {
			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&attitude_param_handles, &attitude_params);
			}

			/* Read Current Measurement */
			if (fds[2].revents & POLLIN) {
				/* Copy esc status data */
				orb_copy(ORB_ID(esc_status), sub_esc_status, &esc_status_raw);

				/* Read esc current data and divide by 100 to get amps*/
				current[0] = 0; //esc_status_raw.esc[0].esc_current/100;
				current[1] = 0; //esc_status_raw.esc[1].esc_current/100;
				current[2] = 0; //esc_status_raw.esc[2].esc_current/100;
				current[3] = 0; //esc_status_raw.esc[3].esc_current/100;			
			}

			/* Read Vehicle status Measurement */
			if (fds[3].revents & POLLIN) {
				/* Copy vehicle status data */
				orb_copy(ORB_ID(vehicle_status), sub_vehicle_status, &vehicle_status_raw);					  
			}

			/* Read GPS measurement */
			if(fds[4].revents & POLLIN) {
				/**/
				orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &rawGPS);
			}
			
			/* Read velocity measurements */
			//rawMeasuredVelocity.inertial_valid = false;
			if(fds[5].revents & POLLIN) {
				/**/
				orb_copy(ORB_ID(vehicle_velocity_meas_inertial), velocity_measured_fd, &rawMeasuredVelocity);
				//printf("Velocity updated\n");
			} 		

			/* Read Vicon measurement */
			if(fds[6].revents & POLLIN) {
				/**/
				orb_copy(ORB_ID(vehicle_vicon_position), vicon_sub_fd, &rawVicon);
			}

			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				/* Without this line here, you don't get the exact 40Hz vicon velocity estimate. 
					Need to rewrite in a similar manner to vel_con */
					
				orb_copy(ORB_ID(vehicle_velocity_meas_inertial), velocity_measured_fd, &rawMeasuredVelocity);
				 				
				
				if (!initialized) {
					gyro_offsets[0] += raw.gyro_rad[0];
					gyro_offsets[1] += raw.gyro_rad[1];
					gyro_offsets[2] += raw.gyro_rad[2];

					acc_offsets[0] += raw.accelerometer_m_s2[0];
					acc_offsets[1] += raw.accelerometer_m_s2[1];
					acc_offsets[2] += raw.accelerometer_m_s2[2];

					mu_init[0] += raw.magnetometer_ga[0];
					mu_init[1] += raw.magnetometer_ga[1];
					mu_init[2] += raw.magnetometer_ga[2];	

					baro_offset += raw.baro_alt_meter;				

					offset_count++;

					if (hrt_absolute_time() > start_time + 3000000l) {
						initialized = true;

						gyro_offsets[0] = gyro_offsets[0]/offset_count;
						gyro_offsets[1] = gyro_offsets[1]/offset_count;
						gyro_offsets[2] = gyro_offsets[2]/offset_count;

						acc_offsets[0] = acc_offsets[0]/offset_count;
						acc_offsets[1] = acc_offsets[1]/offset_count;
						acc_offsets[2] = acc_offsets[2]/offset_count;

						mu_init[0] = mu_init[0]/offset_count;
						mu_init[1] = mu_init[1]/offset_count;
						mu_init[2] = mu_init[2]/offset_count;

						baro_offset = baro_offset/offset_count;

						/* Make the rotation matrix converge to the initial solution faster */					
						filter_b.init(acc_offsets, mu_init);
						filter_a.init(acc_offsets, mu_init);

						prev_timestamp = hrt_absolute_time();
					}

				} 
				else 
				{
					/* Get drag coefficients */
					Cbar(0) = attitude_params.cbar_x;
					Cbar(1) = attitude_params.cbar_y;

					getCurrentCalibParam(attitude_params, current_k,current_c);

					/* Calculate data time difference in seconds */
					dt = (raw.timestamp - prev_timestamp) / 1000000.0f;
					prev_timestamp = raw.timestamp;						
					
					/* All of this should really be in a function */
					/* Body fixed frame velocity measurements */
					vbar(0) = (raw.accelerometer_m_s2[0] + attitude_params.cbar_x_offset)/raw.accelerometer_m_s2[2] /Cbar(0); //0.2569f
					vbar(1) = (raw.accelerometer_m_s2[1] + attitude_params.cbar_y_offset)/raw.accelerometer_m_s2[2] /Cbar(1); //0.4886

					
					baro_alt = raw.baro_alt_meter - baro_offset;
					if (baro_prev_time != raw.baro_timestamp_relative + raw.timestamp)
					{
						use_barometer(attitude_params, raw.baro_timestamp_relative + raw.timestamp, baro_alt, filter_b.att.R, a, baro_data); //FIXME: do we need to consider bias?
					}
					
					if (vbar(0) > 2.0f) //FIXME this is a hack changed later
						vbar(0) = 2.0f;
					if (vbar(0) < -2.0f)
						vbar(0) = -2.0f;

					if (vbar(1) > 2.0f) 
						vbar(1) = 2.0f;
					if (vbar(1) < -2.0f)
						vbar(1) = -2.0f;

					/* Need to capture NaNs and infs */
					if ((isfinite(vbar(0)) == false) || !(isnan(vbar(0)) == false))
						vbar(0) = 0.0f;

					if ((isfinite(vbar(1)) == false) || !(isnan(vbar(1)) == false))
						vbar(1) = 0.0f;	

					vbar(2) = (baro_data(0) - (vbar(0)*filter_b.att.R[6] + vbar(1)*filter_b.att.R[7]))/(filter_b.att.R[8]);

					if ((isfinite(vbar(2)) == false) || !(isnan(vbar(2)) == false))
						vbar(2) = 0.0f;	

					meas_body_vel.meas_vx = vbar(0);
					meas_body_vel.meas_vy = vbar(1);
					meas_body_vel.meas_vz = vbar(2); 

					meas_body_vel.est_vx = filter_b.strapdown_est_vhat(0);
					meas_body_vel.est_vy = filter_b.strapdown_est_vhat(1);
					meas_body_vel.est_vz = filter_b.strapdown_est_vhat(2);
					meas_body_vel.u 		 = filter_b.scaling_u;
					meas_body_vel.baro_alt_est = baro_data(1);
					meas_body_vel.baro_vz_est = baro_data(0);
					meas_body_vel.timestamp = raw.timestamp;

					/* Publish measured and estimated velocities in body frame*/
					if (vel_meas_pub != nullptr) {
						orb_publish(ORB_ID(vehicle_velocity_meas_est_body), vel_meas_pub, &meas_body_vel);
					} else {
						orb_advertise(ORB_ID(vehicle_velocity_meas_est_body), &meas_body_vel);
					}
					

					/* Publish wind velocities*/
					if (wind_vel_pub != nullptr) {
						orb_publish(ORB_ID(wind_estimate), wind_vel_pub, &windVelocity);
					} else {
						orb_advertise(ORB_ID(wind_estimate), &windVelocity);
					}					

					/* Remove bias from gyro measurement */
					omega(0) = raw.gyro_rad[0] - gyro_offsets[0];
					omega(1) = raw.gyro_rad[1] - gyro_offsets[1];
					omega(2) = raw.gyro_rad[2] - gyro_offsets[2];

					/* Accelerometer measurement */
					a(0) = raw.accelerometer_m_s2[0]-acc_offsets[0];
					a(1) = raw.accelerometer_m_s2[1]-acc_offsets[1];
					a(2) = raw.accelerometer_m_s2[2];

					/* Magnetometer measurement, and use current calib */
					mu(0) = raw.magnetometer_ga[0];
					mu(1) = raw.magnetometer_ga[1];
					mu(2) = raw.magnetometer_ga[2];

					/* Poll RC */
					vehicle_rc_poll();

					/* Run the drag offset calibration when velocity measurements are available*/
					if (rawMeasuredVelocity.body_valid)
					{
						if (prev_rc_chan5 > 0 && _v_rc_channels.channels[5] < 0)
						{
							xoff = xvelcalib/vel_calib_count;
							yoff = yvelcalib/vel_calib_count;

							param_set(param_find("ATT_CBAR_XOFF"), &xoff);
							param_set(param_find("ATT_CBAR_YOFF"), &yoff);
						}

						if (_v_rc_channels.channels[5] > 0)
						{
							velocity_offset_calibration(filter_a.vhat, a);
						} else {
							xvelcalib = 0.0f;
							yvelcalib = 0.0f;
							vel_calib_count = 0;
						}
					}	

					prev_rc_chan5 = _v_rc_channels.channels[5];

					/* Apply magnetometer current compensation */
					applyCurrentCompensation(mu, current, current_k, current_c, armed_start_time, attitude_params, vehicle_status_raw);

					/* Update filter */
					filter_b.update(a, omega, vbar, mu, rawVicon, attitude_params, filter_a.vhat, filter_a.what, filter_a.valid, dt);
					filter_a.update(a, omega, filter_b.Rhat, rawMeasuredVelocity, rawGPS, attitude_params, filter_b.strapdown_est_vhat, filter_b.beta_a_filterb, dt);

					/* Publish filter, filter b also publishes the velocity in {A} from filter a */
					filter_b.publish(raw.timestamp, filter_a.vhat, filter_a.what, filter_a.valid);
				}
			}
		}
	}

	thread_running = false;

	return 0;
}

void applyCurrentCompensation(Vector3f &mu, 
			      float * current,
			      float * current_k, 
			      float * current_c, 
			      uint64_t &armed_start_time, 
			      av_estimator_params  attitude_params, 
			      vehicle_status_s vehicle_status_raw)
{
	Vector3f compensation = Vector3f::Zero();

	/* Check for current calib to be enabled and the vehicle to be armed */
	if(attitude_params.curr_calib_en &&
	   vehicle_status_raw.arming_state==vehicle_status_s::ARMING_STATE_ARMED)
	{
		/* Set armed time */
		if(armed_start_time==0)
			armed_start_time = hrt_absolute_time();

		/* Wait for current reading to stabilize (2 seconds) */
		if(hrt_absolute_time() > armed_start_time + 2000000l)
		{
			/* Substract current influence for every motor */
			for(int i = 0; i<4;i++)
			{
				compensation(0) += current_k[i*3]*current[i];
				compensation(1) += current_k[i*3+1]*current[i];
				compensation(2) += current_k[i*3+2]*current[i];
			}

			mu(0) = mu(0);// - compensation(0) - current_c[0];
			mu(1) = mu(1);// - compensation(1) - current_c[1];
			mu(2) = mu(2);// - compensation(2) - current_c[2];
		}
	}
	else
	{
		armed_start_time = 0l;
	}
}

void getCurrentCalibParam(av_estimator_params  attitude_params, float * current_k, float * current_c)
{
	current_k[0]  = attitude_params.magcalib_k0_0;
	current_k[1]  = attitude_params.magcalib_k0_1;
	current_k[2]  = attitude_params.magcalib_k0_2;
	current_k[3]  = attitude_params.magcalib_k1_0;
	current_k[4]  = attitude_params.magcalib_k1_1;
	current_k[5]  = attitude_params.magcalib_k1_2;
	current_k[6]  = attitude_params.magcalib_k2_0;
	current_k[7]  = attitude_params.magcalib_k2_1;
	current_k[8]  = attitude_params.magcalib_k2_2;
	current_k[9]  = attitude_params.magcalib_k3_0;
	current_k[10] = attitude_params.magcalib_k3_1;
	current_k[11] = attitude_params.magcalib_k3_2;

	/* Sum the current calib constants once */
	current_c[0]  = attitude_params.magcalib_c0_0;
	current_c[1]  = attitude_params.magcalib_c0_1;
	current_c[2]  = attitude_params.magcalib_c0_2;
	current_c[0]  += attitude_params.magcalib_c1_0;
	current_c[1]  += attitude_params.magcalib_c1_1;
	current_c[2]  += attitude_params.magcalib_c1_2;
	current_c[0]  += attitude_params.magcalib_c2_0;
	current_c[1]  += attitude_params.magcalib_c2_1;
	current_c[2]  += attitude_params.magcalib_c2_2;
	current_c[0]  += attitude_params.magcalib_c3_0;
	current_c[1]  += attitude_params.magcalib_c3_1;
	current_c[2]  += attitude_params.magcalib_c3_2;

	/* Hack by Moses */
	for (int i=0; i<12; i++)
	{
		current_k[i] = 0;
		current_c[i] = 0;
	}
}

void use_barometer(av_estimator_params  attitude_params, uint64_t cur_baro_time, float baro_alt, float * cur_att, Vector3f &acc, Vector2f &baro_out)
{
	Vector3f e3 = {0.0, 0.0f, 1.0f};
	Matrix3f Rot;

	float g = 9.81f;

	float z_bar_dot, vzdot, baro_dt, dot_beta_az; 
	//uint64_t baro_time;

	baro_alt = -baro_alt;

	baro_dt = (cur_baro_time - baro_prev_time) * 0.000001f;

	baro_prev_time = cur_baro_time;

	Rot(0,0) = cur_att[0]; 
	Rot(0,1) = cur_att[1]; 
	Rot(0,2) = cur_att[2];     

	Rot(1,0) = cur_att[3]; 
	Rot(1,1) = cur_att[4]; 
	Rot(1,2) = cur_att[5];		    

	Rot(2,0) = cur_att[6]; 
	Rot(2,1) = cur_att[7]; 
	Rot(2,2) = cur_att[8];

	Vector3f dd = Rot * acc;

	z_bar_dot = vzbar - attitude_params.baro_k1 * (zbar - baro_alt);
	vzdot = e3.dot(dd) + g - beta_az - attitude_params.baro_k2 * (zbar - baro_alt); //e3*(Rot*acc) +  //e3.transpose*(Rot*acc) +
	dot_beta_az = attitude_params.baro_k3*(zbar - baro_alt);

	zbar += z_bar_dot*baro_dt;
	beta_az += dot_beta_az*baro_dt;
	vzbar += vzdot*baro_dt;

	baro_out(0) = z_bar_dot;
	baro_out(1) = zbar;
}

void 	velocity_offset_calibration(Vector3f veh_vel, Vector3f acc)
{	

	xvelcalib += (veh_vel(0)*Cbar(0)*acc(2) - acc(0));
	yvelcalib += (veh_vel(1)*Cbar(1)*acc(2) - acc(1));
	vel_calib_count += 1;
}
