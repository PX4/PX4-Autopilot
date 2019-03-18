/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file sih.cpp
 * Simulator in Hardware
 *
 * @author Romain Chiappinelli		<romain.chiap@gmail.com>
 *
 * Coriolis g Corporation - January 2019
 */

#include "sih.hpp"

#include <px4_getopt.h>
#include <px4_log.h>

#include <drivers/drv_pwm_output.h>			// to get PWM flags
#include <uORB/topics/vehicle_status.h> 	// to get the HIL status

#include <unistd.h> 			//
#include <string.h>    			//
#include <fcntl.h> 				//
#include <termios.h> 			//

using namespace math;

int Sih::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Simulator in Hardware
This module provide a simulator for quadrotors running fully 
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the mixer.

This simulator publishes the sensors signals corrupted with realistic noise
in order to incorporate the state estimator in the loop.

### Implementation
The simulator implements the equations of motion using matrix algebra. 
Quaternion representation is used for the attitude.
Forward Euler is used for integration.
Most of the variables are declared global in the .hpp file to avoid stack overflow.


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sih", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1024, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Sih::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Sih::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Sih::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sih",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX, //SCHED_PRIORITY_DEFAULT
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Sih *Sih::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Sih *instance = new Sih(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Sih::Sih(int example_param, bool example_flag)
	: ModuleParams(nullptr),
    _loop_perf(perf_alloc(PC_ELAPSED, "sih_execution")),
    _sampling_perf(perf_alloc(PC_ELAPSED, "sih_sampling"))
{
}

void Sih::run()
{

	// to subscribe to (read) the actuators_out pwm
	_actuator_out_sub = orb_subscribe(ORB_ID(actuator_outputs));

	// initialize parameters
	_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update_poll();

	init_variables();
 	init_sensors();	
	// on the AUAVX21: "/dev/ttyS2/" is TELEM2 UART3 --- "/dev/ttyS5/" is Debug UART7 --- "/dev/ttyS4/" is OSD UART8
	// int serial_fd=init_serial_port(); 	 	// init and open the serial port

	const hrt_abstime task_start = hrt_absolute_time();
	_last_run = task_start;
	_gps_time = task_start;
	_serial_time = task_start;

    px4_sem_init(&_data_semaphore, 0, 0);

    hrt_call_every(&_timer_call, LOOP_INTERVAL, LOOP_INTERVAL, timer_callback, &_data_semaphore);

    perf_begin(_sampling_perf);

    while (!should_exit())
    {
        px4_sem_wait(&_data_semaphore);     // periodic real time wakeup

        perf_end(_sampling_perf);
        perf_begin(_sampling_perf);

        perf_begin(_loop_perf);

        inner_loop();   // main execution function

        perf_end(_loop_perf);
    }

    px4_sem_destroy(&_data_semaphore);
	hrt_cancel(&_timer_call); 	// close the periodic timer interruption
	orb_unsubscribe(_actuator_out_sub);
	orb_unsubscribe(_parameter_update_sub);
	// close(serial_fd);

}

// timer_callback() is used as a real time callback to post the semaphore
void Sih::timer_callback(void *sem)
{
	px4_sem_post((px4_sem_t *)sem);
}

// this is the main execution waken up periodically by the semaphore
void Sih::inner_loop()
{
	_now = hrt_absolute_time();
	_dt = (_now - _last_run) * 1e-6f;
	_last_run = _now;

	read_motors();

	generate_force_and_torques();

	equations_of_motion();

	reconstruct_sensors_signals();

	send_IMU();

	if (_now - _gps_time >= 50000) 	// gps published at 20Hz
	{
		_gps_time=_now;
		send_gps();
	}

	// send uart message every 40 ms
	if (_now - _serial_time >= 40000)
	{
		_serial_time=_now;

		publish_sih(); 	// publish _sih message for debug purpose

		// send_serial_msg(serial_fd, (int64_t)(now - task_start)/1000);

		parameters_update_poll(); 	// update the parameters if needed
	}

	_sih.te_us=hrt_absolute_time()-_now; 	// execution time (without delay)

}

void Sih::parameters_update_poll()
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_update_sub, &param_upd);
		updateParams();
		parameters_updated();
	}
}

// store the parameters in a more convenient form
void Sih::parameters_updated()
{

	_T_MAX = _sih_t_max.get();
	_Q_MAX = _sih_q_max.get();
	_L_ROLL = _sih_l_roll.get();
	_L_PITCH = _sih_l_pitch.get();
	_KDV = _sih_kdv.get();
	_KDW = _sih_kdw.get();
	_H0 = _sih_h0.get();

	_LAT0 = (double)_sih_lat0.get()*1.0e-7;
	_LON0 = (double)_sih_lon0.get()*1.0e-7;
	_COS_LAT0=cosl(radians(_LAT0)); 

	_MASS=_sih_mass.get();

	_W_I=Vector3f(0.0f,0.0f,_MASS*CONSTANTS_ONE_G);
	
	_I=diag(Vector3f(_sih_ixx.get(),_sih_iyy.get(),_sih_izz.get()));
	_I(0,1)=_I(1,0)=_sih_ixy.get();
	_I(0,2)=_I(2,0)=_sih_ixz.get();
	_I(1,2)=_I(2,1)=_sih_iyz.get();

	_Im1=inv(_I);

	_mu_I=Vector3f(_sih_mu_x.get(), _sih_mu_y.get(), _sih_mu_z.get());

}

// initialization of the variables for the simulator
void Sih::init_variables()
{
	srand(1234); 	// initialize the random seed once before calling generate_wgn()

	_p_I=Vector3f(0.0f,0.0f,0.0f);
	_v_I=Vector3f(0.0f,0.0f,0.0f);
	_q=Quatf(1.0f,0.0f,0.0f,0.0f);
	_w_B=Vector3f(0.0f,0.0f,0.0f);

	_u[0]=_u[1]=_u[2]=_u[3]=0.0f;

}

void Sih::init_sensors()
{

	_sensor_accel.device_id=1;
	_sensor_accel.error_count=0;		
	_sensor_accel.integral_dt=0;
	_sensor_accel.temperature=T1_C;
	_sensor_accel.scaling=0.0f;

	_sensor_gyro.device_id=1;
	_sensor_gyro.error_count=0;		
	_sensor_gyro.integral_dt=0;
	_sensor_gyro.temperature=T1_C;
	_sensor_gyro.scaling=0.0f;

	_sensor_mag.device_id=1;
	_sensor_mag.error_count=0;		
	_sensor_mag.temperature=T1_C;
	_sensor_mag.scaling=0.0f;		
	_sensor_mag.is_external=false;

	_sensor_baro.error_count=0;
	_sensor_baro.device_id=1;

	_vehicle_gps_pos.fix_type=3; 	// 3D fix
	_vehicle_gps_pos.satellites_used=8;
	_vehicle_gps_pos.heading=NAN;
	_vehicle_gps_pos.heading_offset=NAN;
	_vehicle_gps_pos.s_variance_m_s = 0.5f;
	_vehicle_gps_pos.c_variance_rad = 0.1f;
	_vehicle_gps_pos.eph = 0.9f;
	_vehicle_gps_pos.epv = 1.78f; 
	_vehicle_gps_pos.hdop = 0.7f;
	_vehicle_gps_pos.vdop = 1.1f;	
}

int Sih::init_serial_port()
{
	struct termios uart_config;
	int serial_fd = open(_uart_name, O_WRONLY | O_NONBLOCK | O_NOCTTY);

	if (serial_fd < 0) {
		PX4_ERR("failed to open port: %s", _uart_name);
	}

	tcgetattr(serial_fd, &uart_config); // read configuration

	uart_config.c_oflag |= ONLCR;
	// try to set Bauds rate
	if (cfsetispeed(&uart_config, BAUDS_RATE) < 0 || cfsetospeed(&uart_config, BAUDS_RATE) < 0) {
		PX4_WARN("ERR SET BAUD %s\n", _uart_name);
		close(serial_fd);
	}	

	tcsetattr(serial_fd, TCSANOW, &uart_config); 	// set config

	return serial_fd;
}

// read the motor signals outputted from the mixer
void Sih::read_motors()
{
	struct actuator_outputs_s actuators_out {};

	// read the actuator outputs
	bool updated;
	orb_check(_actuator_out_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_out_sub, &actuators_out);
		for (int i=0; i<NB_MOTORS; i++) 	// saturate the motor signals
			_u[i]=constrain((actuators_out.output[i]-PWM_DEFAULT_MIN)/(PWM_DEFAULT_MAX-PWM_DEFAULT_MIN),0.0f, 1.0f);
	}
}

// generate the motors thrust and torque in the body frame
void Sih::generate_force_and_torques()
{
	_T_B=Vector3f(0.0f,0.0f,-_T_MAX*(+_u[0]+_u[1]+_u[2]+_u[3]));
	_Mt_B=Vector3f(	_L_ROLL*_T_MAX* (-_u[0]+_u[1]+_u[2]-_u[3]),
					_L_PITCH*_T_MAX*(+_u[0]-_u[1]+_u[2]-_u[3]),
						   _Q_MAX * (+_u[0]+_u[1]-_u[2]-_u[3]));

	_Fa_I=-_KDV*_v_I; 		// first order drag to slow down the aircraft
	_Ma_B=-_KDW*_w_B; 		// first order angular damper
}

// apply the equations of motion of a rigid body and integrate one step
void Sih::equations_of_motion()
{
	_C_IB=_q.to_dcm(); 	// body to inertial transformation 

	// Equations of motion of a rigid body
	_p_I_dot=_v_I; 							// position differential
	_v_I_dot=(_W_I+_Fa_I+_C_IB*_T_B)/_MASS; 			// conservation of linear momentum
	_q_dot=_q.derivative1(_w_B); 				// attitude differential
	_w_B_dot=_Im1*(_Mt_B+_Ma_B-_w_B.cross(_I*_w_B));	// conservation of angular momentum

	// fake ground, avoid free fall
	if(_p_I(2)>0.0f && (_v_I_dot(2)>0.0f || _v_I(2)>0.0f))
	{
		_v_I.setZero();
		_w_B.setZero(); 
		_v_I_dot.setZero();
	}
	else
	{
		// integration: Euler forward
		_p_I = _p_I + _p_I_dot*_dt;
		_v_I = _v_I + _v_I_dot*_dt;
		_q = _q+_q_dot*_dt; 	// as given in attitude_estimator_q_main.cpp
		_q.normalize(); 	
		_w_B = _w_B + _w_B_dot*_dt;

	}
}

// reconstruct the noisy sensor signals
void Sih::reconstruct_sensors_signals()
{

	// The sensor signals reconstruction and noise levels are from
	// Bulka, Eitan, and Meyer Nahon. "Autonomous fixed-wing aerobatics: from theory to flight." 
	// In 2018 IEEE International Conference on Robotics and Automation (ICRA), pp. 6573-6580. IEEE, 2018.

	// IMU
	_acc=_C_IB.transpose()*(_v_I_dot-Vector3f(0.0f,0.0f,CONSTANTS_ONE_G))+noiseGauss3f(0.5f,1.7f,1.4f);
	_gyro=_w_B+noiseGauss3f(0.14f,0.07f,0.03f);
	_mag=_C_IB.transpose()*_mu_I+noiseGauss3f(0.02f,0.02f,0.03f);

	// barometer
	float altitude=(_H0-_p_I(2))+generate_wgn()*0.14f; 	// altitude with noise
	_baro_p_mBar=CONSTANTS_STD_PRESSURE_MBAR* 			//  reconstructed pressure in mBar
			powf((1.0f+altitude*TEMP_GRADIENT/T1_K),-CONSTANTS_ONE_G/(TEMP_GRADIENT*CONSTANTS_AIR_GAS_CONST)); 	
	_baro_temp_c=T1_K+CONSTANTS_ABSOLUTE_NULL_CELSIUS+TEMP_GRADIENT*altitude; 	// reconstructed temperture in celcius

	// GPS
	_gps_lat_noiseless=_LAT0+degrees((double)_p_I(0)/CONSTANTS_RADIUS_OF_EARTH);
	_gps_lon_noiseless=_LON0+degrees((double)_p_I(1)/CONSTANTS_RADIUS_OF_EARTH)/_COS_LAT0;
	_gps_alt_noiseless=_H0-_p_I(2);

	_gps_lat=_gps_lat_noiseless+(double)(generate_wgn()*7.2e-6f);  			// latitude in degrees
	_gps_lon=_gps_lon_noiseless+(double)(generate_wgn()*1.75e-5f); 	// longitude in degrees
	_gps_alt=_gps_alt_noiseless+generate_wgn()*1.78f;
	_gps_vel=_v_I+noiseGauss3f(0.06f,0.077f,0.158f);
}

void Sih::send_IMU()
{
	_sensor_accel.timestamp=_now;
	_sensor_accel.x=_acc(0);
	_sensor_accel.y=_acc(1);	
	_sensor_accel.z=_acc(2);
	if (_sensor_accel_pub != nullptr) {
		orb_publish(ORB_ID(sensor_accel), _sensor_accel_pub, &_sensor_accel);
	} else {
		_sensor_accel_pub = orb_advertise(ORB_ID(sensor_accel), &_sensor_accel);
	}

	_sensor_gyro.timestamp=_now;
	_sensor_gyro.x=_gyro(0);
	_sensor_gyro.y=_gyro(1);	
	_sensor_gyro.z=_gyro(2);
	if (_sensor_gyro_pub != nullptr) {
		orb_publish(ORB_ID(sensor_gyro), _sensor_gyro_pub, &_sensor_gyro);
	} else {
		_sensor_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &_sensor_gyro);
	}

	_sensor_mag.timestamp=_now;
	_sensor_mag.x=_mag(0);
	_sensor_mag.y=_mag(1);
	_sensor_mag.z=_mag(2);
	if (_sensor_mag_pub != nullptr) {
		orb_publish(ORB_ID(sensor_mag), _sensor_mag_pub, &_sensor_mag);
	} else {
		_sensor_mag_pub = orb_advertise(ORB_ID(sensor_mag), &_sensor_mag);
	}

	_sensor_baro.timestamp=_now;
	_sensor_baro.pressure=_baro_p_mBar;
	_sensor_baro.temperature=_baro_temp_c;
	if (_sensor_baro_pub != nullptr) {
		orb_publish(ORB_ID(sensor_baro), _sensor_baro_pub, &_sensor_baro);
	} else {
		_sensor_baro_pub = orb_advertise(ORB_ID(sensor_baro), &_sensor_baro);
	}
}

void Sih::send_gps()
{
	_vehicle_gps_pos.timestamp=_now;
	_vehicle_gps_pos.lat=(int32_t)(_gps_lat*1e7); 			// Latitude in 1E-7 degrees	
	_vehicle_gps_pos.lon=(int32_t)(_gps_lon*1e7);	// Longitude in 1E-7 degrees
	_vehicle_gps_pos.alt=(int32_t)(_gps_alt*1000.0f); 	// Altitude in 1E-3 meters above MSL, (millimetres)
	_vehicle_gps_pos.alt_ellipsoid = (int32_t)(_gps_alt*1000); 	// Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
	_vehicle_gps_pos.vel_ned_valid=true;				// True if NED velocity is valid
	_vehicle_gps_pos.vel_m_s=sqrtf(_gps_vel(0)*_gps_vel(0)+_gps_vel(1)*_gps_vel(1));	// GPS ground speed, (metres/sec)
	_vehicle_gps_pos.vel_n_m_s=_gps_vel(0);				// GPS North velocity, (metres/sec)
	_vehicle_gps_pos.vel_e_m_s=_gps_vel(1);				// GPS East velocity, (metres/sec)
	_vehicle_gps_pos.vel_d_m_s=_gps_vel(2);				// GPS Down velocity, (metres/sec)
	_vehicle_gps_pos.cog_rad=atan2(_gps_vel(1),_gps_vel(0));	// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
	if (_vehicle_gps_pos_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_gps_position), _vehicle_gps_pos_pub, &_vehicle_gps_pos);
	} else {
		_vehicle_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_vehicle_gps_pos);
	}
}

void Sih::publish_sih()
{


	_gpos_gt.lat=_gps_lat_noiseless;
	_gpos_gt.lon=_gps_lon_noiseless;
	_gpos_gt.alt=_gps_alt_noiseless;
	_gpos_gt.vel_n=_v_I(0);
	_gpos_gt.vel_e=_v_I(1);
	_gpos_gt.vel_d=_v_I(2);

	if (_gpos_gt_sub != nullptr) {
		orb_publish(ORB_ID(vehicle_global_position_groundtruth), _gpos_gt_sub, &_gpos_gt);
	} else {
		_gpos_gt_sub = orb_advertise(ORB_ID(vehicle_global_position_groundtruth), &_gpos_gt);
	}

	// publish attitude groundtruth
	_att_gt.timestamp=hrt_absolute_time();
	_att_gt.q[0]=_q(0);
	_att_gt.q[1]=_q(1);
	_att_gt.q[2]=_q(2);
	_att_gt.q[3]=_q(3);
	_att_gt.rollspeed=_w_B(0);
	_att_gt.pitchspeed=_w_B(1);
	_att_gt.yawspeed=_w_B(2);
	if (_att_gt_sub != nullptr) {
		orb_publish(ORB_ID(vehicle_attitude_groundtruth), _att_gt_sub, &_att_gt);
	} else {
		_att_gt_sub = orb_advertise(ORB_ID(vehicle_attitude_groundtruth), &_att_gt);
	}

	Eulerf Euler(_q);
	_sih.timestamp=hrt_absolute_time();
	_sih.dt_us=(uint32_t)(_dt*1e6f);
	_sih.euler_rpy[0]=degrees(Euler(0));
	_sih.euler_rpy[1]=degrees(Euler(1));
	_sih.euler_rpy[2]=degrees(Euler(2));
	_sih.omega_b[0]=_w_B(0); 	// wing body rates in body frame
	_sih.omega_b[1]=_w_B(1);
	_sih.omega_b[2]=_w_B(2);
	_sih.p_i_local[0]=_p_I(0); 	// local inertial position
	_sih.p_i_local[1]=_p_I(1);
	_sih.p_i_local[2]=_p_I(2);
	_sih.v_i[0]=_v_I(0); 	// inertial velocity
	_sih.v_i[1]=_v_I(1);
	_sih.v_i[2]=_v_I(2);
	_sih.u[0]=_u[0];
	_sih.u[1]=_u[1];
	_sih.u[2]=_u[2];
	_sih.u[3]=_u[3];
	if (_sih_pub != nullptr) {
		orb_publish(ORB_ID(sih), _sih_pub, &_sih);
	} else {
		_sih_pub = orb_advertise(ORB_ID(sih), &_sih);
	}
} 

void Sih::send_serial_msg(int serial_fd, int64_t t_ms)
{

	char uart_msg[100];

	uint8_t n;
	int32_t GPS_pos[3]; 	// latitude, longitude in 10^-7 degrees, altitude AMSL in mm
	int32_t EA_deci_deg[3]; // Euler angles in deci degrees integers to send to serial
	int32_t throttles[4]; 	// throttles from 0 to 99

	GPS_pos[0]=(int32_t)(_gps_lat_noiseless*1e7); 		// Latitude in 1E-7 degrees	
	GPS_pos[1]=(int32_t)(_gps_lon_noiseless*1e7);			// Longitude in 1E-7 degrees
	GPS_pos[2]=(int32_t)(_gps_alt_noiseless*1000.0f); 	// Altitude in 1E-3 meters above MSL, (millimetres)
	Eulerf Euler(_q);
	EA_deci_deg[0]=(int32_t)degrees(Euler(0)*10.0f); 	// decidegrees
	EA_deci_deg[1]=(int32_t)degrees(Euler(1)*10.0f);
	EA_deci_deg[2]=(int32_t)degrees(Euler(2)*10.0f);
	throttles[0]=(int32_t)(_u[0]*99.0f); 	
	throttles[1]=(int32_t)(_u[1]*99.0f);
	throttles[2]=(int32_t)(_u[2]*99.0f);
	throttles[3]=(int32_t)(_u[3]*99.0f);

	n = sprintf(uart_msg, "T%07lld,P%+010d%+010d%+08d,A%+05d%+05d%+05d,U%+03d%+03d%+03d%+03d\n", 
		t_ms,GPS_pos[0],GPS_pos[1],GPS_pos[2],
		EA_deci_deg[0],EA_deci_deg[1],EA_deci_deg[2],
		throttles[0],throttles[1],throttles[2],throttles[3]);
	write(serial_fd, uart_msg, n);
}

float Sih::generate_wgn() 	// generate white Gaussian noise sample with std=1
{
	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / RAND_MAX;
			float U2 = (float)rand() / RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

Vector3f Sih::noiseGauss3f(float stdx,float stdy, float stdz) 	// generate white Gaussian noise sample with specified std
{
	return Vector3f(generate_wgn()*stdx,generate_wgn()*stdy,generate_wgn()*stdz);
} // there is another wgn algorithm in BlockRandGauss.hpp

int sih_main(int argc, char *argv[])
{
	return Sih::main(argc, argv);
}

// int Sih::pack_float(char* uart_msg, int index, void *value)
// {
// 	uint32_t value_raw=(uint32_t)(value*);

// 	for (int i=3; i>=0; i=i-1) {
// 		buffer[index+i]=(char)(value_raw&0xFF);
// 		value_raw=value_raw>>8;
// 	}

// 	return index+4;	// points to the index for the next value
// }