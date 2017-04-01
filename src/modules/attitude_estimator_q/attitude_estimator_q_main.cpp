/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file attitude_estimator_q_main.cpp
 * 
 * Attitude estimator (quaternion based)
 * 基于四元数的姿态估计
 * 主要相关文章
 * Nonlinear Cmoplementary Filters on the Special Orthogonal Group   --By: Robert Mahony
 * The DCM:IMU Theory
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/geo/geo.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;

class AttitudeEstimatorQ;

namespace attitude_estimator_q
{
AttitudeEstimatorQ *instance;
}

class AttitudeEstimatorQ
{
public:
	/**
	 * Constructor
	 */
	AttitudeEstimatorQ();

	/**
	 * Destructor, also kills task.
	 */
	~AttitudeEstimatorQ();

	/**
	 * Start task.
	 * 开始任务
	 * @return		OK on success. 如果start成功返回OK
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void		print();

private:
	static constexpr float _dt_max = 0.02;
	bool		_task_should_exit = false;		/**< if true, task should exit */
													// 如果为真，任务应该退出
	int		_control_task = -1;			/**< task handle for task */

	int		_sensors_sub = -1;
	int		_params_sub = -1;
	int		_vision_sub = -1;
	int		_mocap_sub = -1;
	int		_airspeed_sub = -1;
	int		_global_pos_sub = -1;
	orb_advert_t	_att_pub = nullptr;
	orb_advert_t	_ctrl_state_pub = nullptr;

	struct {
		param_t	w_acc;
		param_t	w_mag;
		param_t	w_ext_hdg;
		param_t	w_gyro_bias;
		param_t	mag_decl;
		param_t	mag_decl_auto;
		param_t	acc_comp;
		param_t	bias_max;
		param_t	ext_hdg_mode;
		param_t airspeed_mode;
	}		_params_handles;		/**< handles for interesting parameters */

	float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.0f;
	int		_ext_hdg_mode = 0;
	int 	_airspeed_mode = 0;

	Vector<3>	_gyro;
	Vector<3>	_accel;
	Vector<3>	_mag;

	vision_position_estimate_s _vision = {};
	Vector<3>	_vision_hdg;

	att_pos_mocap_s _mocap = {};
	Vector<3>	_mocap_hdg;

	airspeed_s _airspeed = {};

	Quaternion	_q;
	Vector<3>	_rates;
	Vector<3>	_gyro_bias;

	vehicle_global_position_s _gpos = {};
	Vector<3>	_vel_prev;
	Vector<3>	_pos_acc;

	/* Low pass filter for accel/gyro */
	math::LowPassFilter2p _lp_accel_x;
	math::LowPassFilter2p _lp_accel_y;
	math::LowPassFilter2p _lp_accel_z;
	math::LowPassFilter2p _lp_gyro_x;
	math::LowPassFilter2p _lp_gyro_y;
	math::LowPassFilter2p _lp_gyro_z;

	hrt_abstime _vel_prev_t = 0;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

	orb_advert_t	_mavlink_log_pub = nullptr;

	perf_counter_t _update_perf;
	perf_counter_t _loop_perf;

	void update_parameters(bool force);

	int update_subscriptions();

	bool init();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);
};

// 在构造函数里面初始化中进行参数初始化
AttitudeEstimatorQ::AttitudeEstimatorQ() :
	_vel_prev(0, 0, 0),
	_pos_acc(0, 0, 0),
	_lp_accel_x(250.0f, 30.0f),
	_lp_accel_y(250.0f, 30.0f),
	_lp_accel_z(250.0f, 30.0f),
	_lp_gyro_x(250.0f, 30.0f),
	_lp_gyro_y(250.0f, 30.0f),
	_lp_gyro_z(250.0f, 30.0f)
{
	_params_handles.w_acc		= param_find("ATT_W_ACC");
	_params_handles.w_mag		= param_find("ATT_W_MAG");
	_params_handles.w_ext_hdg	= param_find("ATT_W_EXT_HDG");
	_params_handles.w_gyro_bias	= param_find("ATT_W_GYRO_BIAS");
	_params_handles.mag_decl	= param_find("ATT_MAG_DECL");
	_params_handles.mag_decl_auto	= param_find("ATT_MAG_DECL_A");
	_params_handles.acc_comp	= param_find("ATT_ACC_COMP");
	_params_handles.bias_max	= param_find("ATT_BIAS_MAX");
	_params_handles.ext_hdg_mode	= param_find("ATT_EXT_HDG_M");
	_params_handles.airspeed_mode = param_find("FW_ARSP_MODE");
}

/**
 * Destructor, also kills task.
 */
AttitudeEstimatorQ::~AttitudeEstimatorQ()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	attitude_estimator_q::instance = nullptr;
}

int AttitudeEstimatorQ::start()
{
	ASSERT(_control_task == -1);

	/* start the task */ /*POSIX接口的任务启动函数*/
	_control_task = px4_task_spawn_cmd("attitude_estimator_q",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2400,
					   (px4_main_t)&AttitudeEstimatorQ::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void AttitudeEstimatorQ::print()
{
}

void AttitudeEstimatorQ::task_main_trampoline(int argc, char *argv[])
{
	attitude_estimator_q::instance->task_main();
}

void AttitudeEstimatorQ::task_main()
{

#ifdef __PX4_POSIX
	perf_counter_t _perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay"));
	perf_counter_t _perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay"));
	perf_counter_t _perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay"));
#endif

	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));

	_vision_sub = orb_subscribe(ORB_ID(vision_position_estimate));
	_mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));

	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	update_parameters(true);  //参数更新

	hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000); //配置阻塞时间，1ms读取一次sensor_combined的值

		if (_mavlink_fd < 0) {
			_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			PX4_WARN("Q POLL ERROR");
			continue;

		} else if (ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("Q POLL TIMEOUT");
			continue;
		}

		update_parameters(false);  // 参数未更新

		// Update sensors  进行传感器参数更新
		sensor_combined_s sensors;

		if (!orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors)) {
			// Feed validator with recent sensor data
			for (unsigned i = 0; i < (sizeof(sensors.gyro_timestamp) / sizeof(sensors.gyro_timestamp[0])); i++) {

				/* ignore empty fields */
				if (sensors.gyro_timestamp[i] > 0) {
/*
1、进程入口：voidAttitudeEstimatorQ::task_main()
   首先读取gyro、accel、mag的数据
   
	将三类参数分别建立相应的 DataValidatorGroup类来对数据进行处理。

        DataValidatorGroup类： _voter_gyro、_voter_accel、_voter_mag

         调用方法：

         1）首先gyro、accel、mag每次读取数据都是三组三组的读取

         2）先将每组的数据（例如gyro将三个维度的的传感器数据put入（如_voter_gyro.put(...))）
         DataValidatorGroup中，并goto到DataValidator::put函数

         3）在DataValidator函数中计算数据的误差、平均值、并进行滤波。

    滤波入口的put函数：
    
		 val=传感器读取的数据

         _lp=滤波器的系数（lowpass value）

         初始化：由上图可知当第一次读到传感器数据时_mean和_M2置0，_lp=val；

         lp_val= val - _lp

         delta_val= lp_val - _mean

         _mean= (平均值）每次数据读取时，每次val和_lp的差值之和的平均值
         			_mean[i] += delta_val / _event_count

         _M2= （均方根值）delta_val * (lp_val - _mean)的和

         _rms= 均方根值sqrtf(_M2[i] / (_event_count - 1))

         优化滤波器系数：_lp[i]= _lp[i] * 0.5f + val[i] * 0.5f

         _value= val ：get_best()函数的最后调用该结果(通过比较三组数据的confidence大小决定是否选取）。

 		滤波器的confidence函数（信任度函数，貌似模糊控制理论有个隶属函数，应该类似的功能）
 		：返回值是对上N次测量的验证的信任程度，其值在0到1之间，越大越好。返回值是返回上N次测量的误差
 		诊断，用于get_best函数选择最优值，选择的方法如下：

 Switch if:

         1)the confidence is higher and priority is equal or higher

         2)the confidence is no less than 1% different and the priority is higher
2、根据_voter_gyro、_voter_accel、_voter_mag三个参数的failover(失效备援)_count函数判断是否存在数据获取失误问题，并通过mavlink协议显示错误原因。

3、根据_voter_gyro、_voter_accel、_voter_mag三个参数的get_vibration_factor函数判断是否有震动现象，返回值是float型的RSM值，其代表振动的幅度大小。
*/


		/*  1）首先gyro、accel、mag每次读取数据都是三组三组的读取  */
					float gyro[3];

					for (unsigned j = 0; j < 3; j++) {
						if (sensors.gyro_integral_dt[i] > 0) {
							gyro[j] = (double)sensors.gyro_integral_rad[i * 3 + j] / (sensors.gyro_integral_dt[i] / 1e6);
										//gyro_rad_s[i * 3 + 0]=vect(0)
						} else {
							/* fall back to angular rate */
							// 退回到角速度
							gyro[j] = sensors.gyro_rad_s[i * 3 + j];
						}
					}
				/*
				*  2）先将每组的数据（例如gyro将三个维度的的传感器数据put入（如_voter_gyro.put(...))）
        		*  DataValidatorGroup中，并goto到DataValidator::put函数
				*/
					_voter_gyro.put(i, sensors.gyro_timestamp[i], &gyro[0], sensors.gyro_errcount[i], sensors.gyro_priority[i]);
				}

				/* ignore empty fields */
				if (sensors.accelerometer_timestamp[i] > 0) {
					_voter_accel.put(i, sensors.accelerometer_timestamp[i], &sensors.accelerometer_m_s2[i * 3],
							 sensors.accelerometer_errcount[i], sensors.accelerometer_priority[i]);
				}

				/* ignore empty fields */
				if (sensors.magnetometer_timestamp[i] > 0) {
					_voter_mag.put(i, sensors.magnetometer_timestamp[i], &sensors.magnetometer_ga[i * 3],
						       sensors.magnetometer_errcount[i], sensors.magnetometer_priority[i]);
				}
				
			}

			// Get best measurement values
			// 获得最佳测量结果
			hrt_abstime curr_time = hrt_absolute_time();
			_gyro.set(_voter_gyro.get_best(curr_time, &best_gyro));
			_accel.set(_voter_accel.get_best(curr_time, &best_accel));
			_mag.set(_voter_mag.get_best(curr_time, &best_mag));

			if (_accel.length() < 0.01f || _mag.length() < 0.01f) {
				warnx("WARNING: degenerate accel / mag!"); // 加速度计或磁力计向量模长<0.01
				continue;
			}

			_data_good = true;

			if (!_failsafe) {
				uint32_t flags = DataValidator::ERROR_FLAG_NO_ERROR;

#ifdef __PX4_POSIX
				perf_end(_perf_accel);
				perf_end(_perf_mpu);
				perf_end(_perf_mag);
#endif

				if (_voter_gyro.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_gyro.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Gyro #%i failure :%s%s%s%s%s!",
									  _voter_gyro.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_voter_accel.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_accel.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Accel #%i failure :%s%s%s%s%s!",
									  _voter_accel.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_voter_mag.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_mag.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Mag #%i failure :%s%s%s%s%s!",
									  _voter_mag.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_failsafe) {
					mavlink_and_console_log_emergency(_mavlink_fd, "SENSOR FAILSAFE! RETURN TO LAND IMMEDIATELY");

				}
			}

			if (sensors.magnetometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
				_mag(0) = sensors.magnetometer_ga[0];
				_mag(1) = sensors.magnetometer_ga[1];
				_mag(2) = sensors.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_DEBUG("WARNING: degenerate mag!");
					continue;
				}
			}

			_data_good = true;
		}

		// 更新视觉位置估计与动作捕捉的航向 Update vision and motion capture heading
		bool vision_updated = false;
		orb_check(_vision_sub, &vision_updated);

		bool mocap_updated = false;
		orb_check(_mocap_sub, &mocap_updated);

		if (vision_updated) {
			orb_copy(ORB_ID(vision_position_estimate), _vision_sub, &_vision);
			math::Quaternion q(_vision.q);

			math::Matrix<3, 3> Rvis = q.to_dcm();
			math::Vector<3> v(1.0f, 0.0f, 0.4f);

			// Rvis is Rwr (robot respect to world) while v is respect to world.
			// Rvis是vision的旋转矩阵 机体到地球 v是地球中的向量 Vw
			// Hence Rvis must be transposed having (Rwr)' * Vw
			// 机体向量到地球 R'*Vw
			// Rrw * Vw = vn. This way we have consistency
			_vision_hdg = Rvis.transposed() * v;
		}

		if (mocap_updated) {
			orb_copy(ORB_ID(att_pos_mocap), _mocap_sub, &_mocap);
			math::Quaternion q(_mocap.q);
			math::Matrix<3, 3> Rmoc = q.to_dcm();

			math::Vector<3> v(1.0f, 0.0f, 0.4f);

			// Rmoc is Rwr (robot respect to world) while v is respect to world.
			// Hence Rmoc must be transposed having (Rwr)' * Vw
			// Rrw * Vw = vn. This way we have consistency
			_mocap_hdg = Rmoc.transposed() * v;
		}

		// Update airspeed 更新空速
		bool airspeed_updated = false;
		orb_check(_airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		}

		// Check for timeouts on data
		if (_ext_hdg_mode == 1) {
			_ext_hdg_good = _vision.timestamp > 0 && (hrt_elapsed_time(&_vision.timestamp) < 500000);

		} else if (_ext_hdg_mode == 2) {
			_ext_hdg_good = _mocap.timestamp > 0 && (hrt_elapsed_time(&_mocap.timestamp) < 500000);
		}


	/* 然后就是获取机体的速度，通过速度 计算机体的加速度。 */
		bool gpos_updated;
		orb_check(_global_pos_sub, &gpos_updated);
	   /*
		* 订阅者可以用来检查一个主题在发布者上一次更新数据后，有没有订阅者调用过orb_copy来
		* 接收、处理过
		* 如果最后一次更新的数据被获取了，检测到并设置updated为true；
		* 返回值OK表示检测成功，错误返回ERROR
		*/
		if (gpos_updated) {
			/*
			 *	从订阅的主题vehicle_global_position中获取飞行器位置信息数据保存到_gpos中
			 *	 _global_pos_sub是订阅主题返回的句柄
			 */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_gpos);

		/*
		首先检测是否配置了自动磁偏角获取，如果配置了则用当前的经纬度（longitude and latitude） 
		通过get_mag_declination(_gpos.lat,_gpos.lon)函数获取当前位置的磁偏角
		（magnetic declination(_mag_decl)）
		*/
			if (_mag_decl_auto && _gpos.eph < 20.0f && hrt_elapsed_time(&_gpos.timestamp) < 1000000) {
				/* set magnetic declination automatically 自动设置磁偏移 */
				update_mag_declination(math::radians(get_mag_declination(_gpos.lat, _gpos.lon))); //从经纬度
			}
		}
		
		if (_acc_comp && _gpos.timestamp != 0 && hrt_absolute_time() < _gpos.timestamp + 20000 && _gpos.eph < 5.0f && _inited) {
			/* position data is actual位置数据是真实的 */
			if (gpos_updated) {
				Vector<3> vel(_gpos.vel_n, _gpos.vel_e, _gpos.vel_d);

				/* velocity updated */
				if (_vel_prev_t != 0 && _gpos.timestamp != _vel_prev_t) {
					float vel_dt = (_gpos.timestamp - _vel_prev_t) / 1000000.0f;
					/* calculate acceleration in body frame 
					 * 计算飞机在机体坐标系中的加速度:速度之差除时间*/
					_pos_acc = _q.conjugate_inversed((vel - _vel_prev) / vel_dt);
				}

				_vel_prev_t = _gpos.timestamp; //更新时间
				_vel_prev = vel;
			} 

		} else {
			/* position data is outdated, reset acceleration */
			// 位置数据过时，重置加速度
			_pos_acc.zero();
			_vel_prev.zero();
			_vel_prev_t = 0;
		}

		/* time from previous iteration 时间迭代 */
		hrt_abstime now = hrt_absolute_time();
		float dt = (last_time > 0) ? ((now  - last_time) / 1000000.0f) : 0.00001f;
		last_time = now;

		if (dt > _dt_max) {
			dt = _dt_max;
		}

		// 如果
		if (!update(dt)) {   
			continue;
		}

		// 将_q转换成欧拉角euler并发布
		Vector<3> euler = _q.to_euler();

		struct vehicle_attitude_s att = {};
		att.timestamp = sensors.timestamp;

		att.roll = euler(0);  //获取的欧拉角赋值给roll、pitch、yaw  
		att.pitch = euler(1);
		att.yaw = euler(2);

		att.rollspeed = _rates(0);  //获取roll、pitch、yaw得角速度
		att.pitchspeed = _rates(1);
		att.yawspeed = _rates(2);

		for (int i = 0; i < 3; i++) {
			//获取导航坐标系的重力加速度
			att.g_comp[i] = _accel(i) - _pos_acc(i); 
		}

		/* copy offsets 复制偏移量 */
		memcpy(&att.rate_offsets, _gyro_bias.data, sizeof(att.rate_offsets));

		Matrix<3, 3> R = _q.to_dcm();

		/* copy rotation matrix  复制旋转矩阵 */
		memcpy(&att.R[0], R.data, sizeof(att.R));
		att.R_valid = true;
		memcpy(&att.q[0], _q.data, sizeof(att.q));
		att.q_valid = true;

		/* the instance count is not used here */
		int att_inst;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &att_inst, ORB_PRIO_HIGH);

		{
			struct control_state_s ctrl_state = {};

			ctrl_state.timestamp = sensors.timestamp;

			/* attitude quaternions for control state */
			// 用于状态控制的姿态四元数
		// 在姿态控制 attitude_control中会调用此四元数
			ctrl_state.q[0] = _q(0);
			ctrl_state.q[1] = _q(1);
			ctrl_state.q[2] = _q(2);
			ctrl_state.q[3] = _q(3);

			ctrl_state.x_acc = _accel(0);
			ctrl_state.y_acc = _accel(1);
			ctrl_state.z_acc = _accel(2);

			/* attitude rates for control state */
			ctrl_state.roll_rate = _rates(0);

			ctrl_state.pitch_rate = _rates(1);

			ctrl_state.yaw_rate = _rates(2);

			ctrl_state.airspeed_valid = false;

			if (_airspeed_mode == control_state_s::AIRSPD_MODE_MEAS) {
				// use measured airspeed
				// 空速: 因为这里没有考虑到风的影响，所以直接使用测得的空速
				if (PX4_ISFINITE(_airspeed.indicated_airspeed_m_s) && hrt_absolute_time() - _airspeed.timestamp < 1e6
				    && _airspeed.timestamp > 0) {
					ctrl_state.airspeed = _airspeed.indicated_airspeed_m_s;
					ctrl_state.airspeed_valid = true;
				}
			}

			else if (_airspeed_mode == control_state_s::AIRSPD_MODE_EST) {
				// use estimated body velocity as airspeed estimate
				if (hrt_absolute_time() - _gpos.timestamp < 1e6) {
					ctrl_state.airspeed = sqrtf(_gpos.vel_n * _gpos.vel_n + _gpos.vel_e * _gpos.vel_e + _gpos.vel_d * _gpos.vel_d);
					ctrl_state.airspeed_valid = true;
				}

			} else if (_airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED) {
				// do nothing, airspeed has been declared as non-valid above, controllers
				// will handle this assuming always trim airspeed
			}

			/* the instance count is not used here */
			int ctrl_inst;
			/* publish to control state topic */
			orb_publish_auto(ORB_ID(control_state), &_ctrl_state_pub, &ctrl_state, &ctrl_inst, ORB_PRIO_HIGH);
		}

		{
			//struct estimator_status_s est = {};

			//est.timestamp = sensors.timestamp;

			/* the instance count is not used here */
			//int est_inst;
			/* publish to control state topic */
			// TODO handle attitude states in position estimators instead so we can publish all data at once
			// or we need to enable more thatn just one estimator_status topic
			// orb_publish_auto(ORB_ID(estimator_status), &_est_state_pub, &est, &est_inst, ORB_PRIO_HIGH);
		}
	}

#ifdef __PX4_POSIX
	perf_end(_perf_accel);
	perf_end(_perf_mpu);
	perf_end(_perf_mag);
#endif

	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_vision_sub);
	orb_unsubscribe(_mocap_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_global_pos_sub);
}

void AttitudeEstimatorQ::update_parameters(bool force)
{
	bool updated = force;

	if (!updated) {
		orb_check(_params_sub, &updated);
	}

	if (updated) {
		parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

		param_get(_params_handles.w_acc, &_w_accel);
		param_get(_params_handles.w_mag, &_w_mag);
		param_get(_params_handles.w_ext_hdg, &_w_ext_hdg);
		param_get(_params_handles.w_gyro_bias, &_w_gyro_bias);
		float mag_decl_deg = 0.0f;
		param_get(_params_handles.mag_decl, &mag_decl_deg);
		update_mag_declination(math::radians(mag_decl_deg)); //更新磁偏移
		int32_t mag_decl_auto_int;
		param_get(_params_handles.mag_decl_auto, &mag_decl_auto_int);
		_mag_decl_auto = mag_decl_auto_int != 0;    //自动获取磁偏移
		int32_t acc_comp_int;
		param_get(_params_handles.acc_comp, &acc_comp_int);
		_acc_comp = acc_comp_int != 0;
		param_get(_params_handles.bias_max, &_bias_max);
		param_get(_params_handles.ext_hdg_mode, &_ext_hdg_mode);
		param_get(_params_handles.airspeed_mode, &_airspeed_mode);
	}
}

		/*		对四元数向量_q进行初始化赋值或者更新		*/
		/*   首先判断是否是第一次进入该函数，第一次进入该函数先进入init函数初始化  */
bool AttitudeEstimatorQ::init()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame 
	//'k'是地球坐标系的X轴(向下)在机体坐标系的对应，与K垂直

	Vector<3> k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	//i是地球坐标系的X轴在机体坐标系的对应，与K垂直

	Vector<3> i = (_mag - k * (_mag * k));  // 与dcm-IMU ； Theory 中强制正交化不同
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	//‘j’ 是地理坐标系的Y轴（向东）在机体坐标系的对应，与‘k’、‘i’都垂直

	Vector<3> j = k % i;

	// Fill rotation matrix  构造由加速度计和陀螺仪生成的旋转矩阵Ry(对于ECF)
	Matrix<3, 3> R;
	R.set_row(0, i);
	R.set_row(1, j);
	R.set_row(2, k);

	// Convert to quaternion
	_q.from_dcm(R);

	// Compensate for magnetic declination
	Quaternion decl_rotation;
	decl_rotation.from_yaw(_mag_decl);
	_q = decl_rotation * _q;

	_q.normalize();

	if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	    PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
	    _q.length() > 0.95f && _q.length() < 1.05f) {
		_inited = true;

	} else {
		_inited = false;
	}

	return _inited;
}

bool AttitudeEstimatorQ::update(float dt)
{
	if (!_inited) { //未初始化

		if (!_data_good) { 
			return false; // 已初始化 数据好  回到初始化
		}
		return init();
	}

	Quaternion q_last = _q; // 初始化得到的四元数

	// Angular rate of correction
	Vector<3> corr;

		/*  如果不是第一次进入该函数，则判断是使用什么mode做修正的，
		    比如vision、mocap、acc和mag（DJI精灵4用的视觉壁障应该就是这个vision)
		    hdg就是heading		*/
		/*
		_ext_hdg_mode== 1、2时都是利用vision数据和mocap数据对gyro(陀螺仪)数据进行
		修正，下面的global frame就是所谓的earthframe。
		*/
	if (_ext_hdg_mode > 0 && _ext_hdg_good) {
		if (_ext_hdg_mode == 1) {
			// Vision heading correction 视觉朝向矫正
			// Project heading to global frame and extract XY component 
			// 朝向地球坐标系的投影，提取出x.y元素
			Vector<3> vision_hdg_earth = _q.conjugate(_vision_hdg);
			float vision_hdg_err = _wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// Project correction to body frame
			//n系到b系
			corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -vision_hdg_err)) * _w_ext_hdg;
		}

		if (_ext_hdg_mode == 2) {
			// Mocap heading correction
			// Project heading to global frame and extract XY component
			Vector<3> mocap_hdg_earth = _q.conjugate(_mocap_hdg);
			float mocap_hdg_err = _wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mocap_hdg_err)) * _w_ext_hdg;
		}
	}
		/*   _ext_hdg_mode== 0  利用磁力计修正  */
	if (_ext_hdg_mode == 0  || !_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		// 磁力计校正
		// 将磁场向量投影到地球坐标系，并提取X.Y元素
		Vector<3> mag_earth = _q.conjugate(_mag);  //b系到n系
		// Vector<3>在/src/lib.../math/Q.hpp   Vector<3>做前缀类似于int 定义变量
		/*
		先将归一化的_q的旋转矩阵R（b系转n系）乘以_mag向量（以自身机体坐标系为视角
		看向北方的向量表示），得到n系（NED坐标系）下的磁力计向量。 
		*/
		float mag_err = _wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl); 
		// _mag_decl是根据经纬度获得的偏移
		/*
		只考虑Vector<3> mag_earth中的前两维的数据mag_earth(1)和mag_earth(0)
		（即x、y，忽略z轴上的偏移），通过arctan(反正切)（mag_earth(1),mag_earth(0)）得到
		的角度和前面根据经纬度获取的磁偏角做差值得到纠偏误差角度mag_err
		_wrap_pi函数是用于限定结果-pi到pi的函数
		*/
		// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mag_err)) * _w_mag;//n系到b系
		// _w_mag为mag的权重
		/*
		计算corr值等于单位化的旋转矩阵R（b系转n系）的转置（可以理解为 R（n系转b系））乘以
		（0,0，-mag_err），相当于机体坐标系绕地理坐标系N轴（Z轴）转动arctan(反正切)（ 
		mag_earth(1), mag_earth(0)）度。
		*/
	}

	_q.normalize();

	/*
	 * 加速度计修正
	 * 首先就是把归一化的n系重力加速度通过旋转矩阵R左乘旋转到b系，即k为归一化的
	 * 旋转矩阵R（b-e）的第三行
	*/
	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector<3> k = _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros 优化后的版本
	Vector<3> k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

						
//  {k%（_accel“加速度计的测量值”-位移加速度）的单位化）<约等于重力加速度g>}*权重
	/*
	关于这个“_accel-_pos_acc”的含义：
	总的受到合力的方向（_accel）减去机体加速度方向（_pos_acc）得到g的方向，即总加速度
	（加速度获取）减去机体运动加速度（第五部分）获取重力加速度，然后姿态矩阵的不是行就是
	列来与纯重力加速度来做叉积，算出误差。因为运动加速度是有害的干扰，必须减掉。
	算法的理论基础是[0,0,1]与姿态矩阵相乘。该差值获取的重力加速度的方向是导航坐标系下的 
	z轴，加上运动加速度之后，总加速度的方向就不是与导航坐标系的天或地平行了，所以要消除
	这个误差，即“_accel-_pos_acc”。然后叉乘z轴向量得到误差，进行校准 。
	*/
	
	corr += (k % (_accel - _pos_acc).normalized()) * _w_accel;

	// Gyro bias estimation

	//if (_gyro.length() < 1.0f) {
	//	_gyro_bias += corr * (_w_gyro_bias * dt);
	//}
	_gyro_bias += corr * (_w_gyro_bias * dt); 

	for (int i = 0; i < 3; i++) {
		_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
	}
  	//使用修正的数据更新四元数，并把_rates和_gyro_bias置零便于下次调用时使用。
	_rates = _gyro + _gyro_bias;  //得到经过修正后的角速度

	// Feed forward gyro
	corr += _rates;  //PI控制器的体现

	// Apply correction to state
	_q += _q.derivative(corr) * dt; //微分方程离散化的思想
	/*
	平时一大推的论文和期刊上面都是用的omga *Q 的形式，而这里的代码实现确是用的
	Q * omga的形式，所以构造的4*4矩阵的每一列的符号就不一样了。
	*/

	/*
	corr包含磁力计修正、加速度计修正、（vision、mocap修正）、gyro(陀螺仪)中测量到的
	角速度偏转量，且因为corr为update函数中定义的变量，所以每次进入update函数中时会刷新
	corr变量的数据； _rate也会刷新其中的数据，含义为三个姿态角的角速度（修正后）；
	_q为外部定义的变量，在这个函数中只会+=不会重新赋值，如果计算出现错误会返回上一次计算出
	的_q。
	*/

	// Normalize quaternion
	_q.normalize();

	if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	      PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) {
		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		return false;
	}

	return true;
}

//更新磁偏移量
void AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	// 在不改变估计值的情况下应用初始经纬度的磁偏差或小的旋转
	if (!_inited || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination; //declination 磁偏差

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		// 立刻旋转现在的估计阵以避免陀螺仪偏移增加
		Quaternion decl_rotation;
		decl_rotation.from_yaw(new_declination - _mag_decl);
		_q = decl_rotation * _q;
		_mag_decl = new_declination;
	}
}
	/*	main函数原型	*/	
int attitude_estimator_q_main(int argc, char *argv[])
{
	if (argc < 1) {
		warnx("usage: attitude_estimator_q {start|stop|status}");
		return 1;
	}
	
	/* 调用姿态估计的启动函数start() */
	if (!strcmp(argv[1], "start")) {
		if (attitude_estimator_q::instance != nullptr) {
			warnx("already running");
			return 1;
		}

		attitude_estimator_q::instance = new AttitudeEstimatorQ;  //赋予新的姿态估计默认值

		if (attitude_estimator_q::instance == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != attitude_estimator_q::instance->start()) {
			delete attitude_estimator_q::instance;
			attitude_estimator_q::instance = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (attitude_estimator_q::instance == nullptr) {
			warnx("not running");
			return 1;
		}

		delete attitude_estimator_q::instance;
		attitude_estimator_q::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (attitude_estimator_q::instance) {
			attitude_estimator_q::instance->print();
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}