/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 * 多旋翼位置控制
 *
 * Original publication for the desired attitude generation:
 * 目标姿态的产生的来源
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * 参考文献： Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 * 同时使用了L1导航
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * 该控制器有两个回路：位置误差的P回路以及速度误差的PID回路。
 * 速度控制器的输出是分割到推力方向的推力向量（例如，多旋翼方位的旋转矩阵）以及推力模块（例如多旋翼推力自身）
 * 该位置控制器没有使用欧拉角，欧拉角只是为了更为人性化的控制与记录日志

 * 在这个文件中 position | pos 这里位置指的是 水平面，x y
 * Altitude 指的就是垂直方向 z
 * 爬升率指的是垂直方向z的速度
 * Velocity这个速度指的是 水平面上x y的速度
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f

/**
 * Multicopter position control app start / stop handling function
 * 多旋翼位置控制的应用程序开始/结束处理函数
 * 
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);
class MulticopterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 * 构造函数
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 * 构造函数，同时杀死任务
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 * 开始任务
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	bool	_task_should_exit;		/**< if true, task should exit */
	int		_control_task;				/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */

	orb_advert_t	_att_sp_pub;				/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos; // 经纬度单位为弧度
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control;
	bool _run_alt_control;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp; // 位置设定值，地面坐标系上x,y，z方向位置
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity 当前速度的微分 */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;		/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
								// 判断已经着陆
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */
										//  速度控制模式中的上一个循环(control_state.flag_control_velocity_enabled)

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 * 检测订阅的话题是否更新
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 * 更新参考系用于本地位置的投影
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position.
	 * 重置位置设定值到当前位置
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 * 仅当_reset_pos_sp标志位置位时进行位置设定值复位。
	 * 一般逻辑是首先再飞行状态中"激活此标志位，当切换到位置控制模式应该保持上一刻的位置
	 * 一旦切换到位置控制模式，上一刻的位置就立即储存起来
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 * 将高度设定值复位到当前高度
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 * 仅当 _reset_alt_sp标志位置位时才会发生。
	 * 一般逻辑是这会发生在reset_pos_sp()之后
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 * 检测位置设定值是否距离当前位置太远，并在需要时进行调节
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 * 使用手动控制模式设置位置设定值
	 */
	void		control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 * 使用外部控制模式设置位置设定值
	 */
	void		control_offboard(float dt);

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

	/**
	 * Set position setpoint for AUTO
	 * 设置AUTO模式的位置设定值
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 * 旋转使用气压计高度或AMSL(平均海平面以上)高度
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
{

MulticopterPositionControl	*g_control;
}

MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(0),
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	control_vel_enabled_prev(false)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();

	_R.identity();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P"); 
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");

	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		// 任务每100ms唤醒一次
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

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated); //检测订阅的话题是否更新

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min); //将param_handle的地址作为参数的地址
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land); //将角度转换为弧度

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);  // 位置速度PID
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now  暂时使用最大Z轴向上速度*/
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1 s from full speed is feasible
		 */
		 // 增加最大的水平加速度，这样的话尚可能在1秒内令飞机从最大速度停下来
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max);
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		// 多旋翼姿态控制参数
		// 手动控制模式下的限幅
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		// 起飞以及着陆的速度不宜超过最大设定值
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated); //飞行器的整体性能_vehicle_status_sub.机型、状态……

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
	 	// 根据飞行器是否为VTOL设置正确的uORB_ID主题
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_ctrl_state_sub, &updated); // 控制状态 control_state.四元数、三轴加速度、速度、位置、角速度……

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

		/* get current rotation matrix and euler angles from control state quaternions */
		// 从控制状态四元数中获得当前旋转矩阵和欧拉角
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]); //由姿态解算过程获得
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2); //单独提出了偏航角
	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated); //飞行模式control_mode.自稳、定高……

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated); //手动控制 manual_control_setpoint.遥控器输出，模式切换

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz, float dy)
{
	if (ctl > dz) {
		return dy + (ctl - dz) * (1.0f - dy) / (end - dz);

	} else if (ctl < -dz) {
		return -dy + (ctl + dz) * (1.0f - dy) / (end - dz);

	} else {
		return ctl * (dy / dz);
	}
}

float
MulticopterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;

	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}


// 这种方式将位置转换为经纬度和高度， 
// 然后用位置估计参数来更新经纬度和高度，接着转换回位置参考点
// GPS数据转换的方式
void
MulticopterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {//参考时间帧改变则更新参考
		double lat_sp, lon_sp; // 单位为度
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) { //获取当前的位置和高度参考数值
			/* calculate current position setpoint in global frame */
			// 计算全球坐标系(GPS测得值)中的当前位置设定值，经纬度用角度表示
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2); //高度设定值
		}

		/* update local projection reference */
		// 更新本地投影的参考平面  
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon); // 度转换成弧度 ，更新时间戳
		_ref_alt = _local_pos.ref_alt; // 参考高度赋值

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			// 将map_projection_reproject()中算得的lat_sp、lon_sp投影到新参考平面中
			// 将全球坐标系中的坐标(上面计算的位置设定值)投影到新的参考平面(本地参考系)中 
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;

		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		// 我们在主函数中设置了一种逻辑：选择速度设定值，因此在切换到速度控制模式时姿态设定值是持续的，
		// 因此我们不需要使用很特别的方法去对高度进行复位.
		// 在位置控制模式中，除非飞行器减速，否则无论如何，位置都将会复位
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		// 我们在主函数中设置了一种逻辑：选择速度设定值，因此在切换到速度控制模式时姿态设定值是持续的，
		// 因此我们不需要使用很特别的方法去对高度进行复位
		_pos_sp(2) = _pos(2);
	}
}

void
MulticopterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
MulticopterPositionControl::control_manual(float dt)
{
	math::Vector<3> req_vel_sp; // X,Y in local frame and Z in global (D), in [-1,1] normalized range
								// X,Y在本地坐标系，Z在全球坐标系（D指向下）
	req_vel_sp.zero();

	if (_control_mode.flag_control_altitude_enabled) {
		/* set vertical velocity setpoint with throttle stick */
		/* 将自稳模式的油门杆转换成控制垂直速度设定值(以中间速度为0，
		 * 往上拨速度向上，往下拨速度向下，速度大小与拨动幅度成正比) 
		 */ 
		req_vel_sp(2) = -scale_control(_manual.z - 0.5f, 0.5f, _params.alt_ctl_dz, _params.alt_ctl_dy); // D
	}//尺度控制器（将0-1转换为-1——1）

	if (_control_mode.flag_control_position_enabled) {
		/* set horizontal velocity setpoint with roll/pitch stick */
		// 根据横滚/俯仰遥杆设置水平速度设定值
		req_vel_sp(0) = _manual.x;
		req_vel_sp(1) = _manual.y;
	}

	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		// 需要时，将高度设定值复位到当前高度
		reset_alt_sp();//复位高度设定值  
	}

	if (_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		// 需要时，将位置设定值复位到当前位置
		reset_pos_sp();
	}

	/* limit velocity setpoint */
	// 限制速度设定值
	float req_vel_sp_norm = req_vel_sp.length();//速度向量的长度

	if (req_vel_sp_norm > 1.0f) {
		req_vel_sp /= req_vel_sp_norm;
	}

	/* _req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw */
	// 将_req_vel_sp缩放到0~1之间，缩放到最大的速度并绕偏航轴旋转
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body); //仅绕偏航轴旋转的DCM
	math::Vector<3> req_vel_sp_scaled = R_yaw_sp * req_vel_sp.emult(
			_params.vel_cruise); // in NED and scaled to actual velocity
								 // 在NED坐标系下，还原到真实的速度  

	/*
	 * assisted velocity mode: user controls velocity, but if	velocity is small enough, position
	 * hold is activated for the corresponding axis
	 */

	/* horizontal axes */
	// 水平轴
	if (_control_mode.flag_control_position_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(0)) < _params.hold_xy_dz && fabsf(req_vel_sp(1)) < _params.hold_xy_dz) {
			if (!_pos_hold_engaged) {
				if (_params.hold_max_xy < FLT_EPSILON || (fabsf(_vel(0)) < _params.hold_max_xy
						&& fabsf(_vel(1)) < _params.hold_max_xy)) {
					_pos_hold_engaged = true;

				} else {
					_pos_hold_engaged = false;
				}
			}

		} else {
			_pos_hold_engaged = false;
		}

/////////////以上为辅助速度模式：用户控制速度，但是如果速度很小，那么相应轴上的位置保持不变/////////////////  
/////////////以下为速度设定值，作为此函数的输出///////////////  
		/* set requested velocity setpoint */
		if (!_pos_hold_engaged) {//不需要位置锁定(辅助速度模式)  
			_pos_sp(0) = _pos(0);
			_pos_sp(1) = _pos(1);
			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
									/* 用于速度设定而不是位置设定 */  
			_vel_sp(0) = req_vel_sp_scaled(0);
			_vel_sp(1) = req_vel_sp_scaled(1);
		}
	}

	/* vertical axis */
////////////以下为垂直轴辅助速度模式：用户控制速度，但是如果速度很小，那么相应轴上的位置保持不变/////////////////  
	if (_control_mode.flag_control_altitude_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(2)) < FLT_EPSILON) {
			if (!_alt_hold_engaged) {
				if (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z) {
					_alt_hold_engaged = true;

				} else {
					_alt_hold_engaged = false;
				}
			}

		} else {
			_alt_hold_engaged = false;
		}

		/* set requested velocity setpoint */
		if (!_alt_hold_engaged) {
			_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
			_vel_sp(2) = req_vel_sp_scaled(2);
			_pos_sp(2) = _pos(2);
		}
	}
}

void
MulticopterPositionControl::control_offboard(float dt)
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated); //之前、现在、下一刻的位置

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}

	if (_pos_sp_triplet.current.valid) {
		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			//控制模式-位置控制使能&&当前位置设定值合法，那么进行位置控制
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;

		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			// 速度控制
			// 需要时将位置设定值复位到当前位置
			reset_pos_sp();

			/* set position setpoint move rate */
			// 设置位置设定值的移动速度
			_vel_sp(0) = _pos_sp_triplet.current.vx;
			_vel_sp(1) = _pos_sp_triplet.current.vy;

			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
									  // 需要使用的速度设定值，而不是位置设定值
		}
////////////yaw姿态设定///////////  
		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;

		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;//使用偏航角速度更新机体的偏航角度
		}
/////////////垂直轴设定/////////// 
		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.position_valid) {
			/* Control altitude 高度控制*/
			_pos_sp(2) = _pos_sp_triplet.current.z;

		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* reset alt setpoint to current altitude if needed */
			// 需要时将高度设定值复位到当前高度
			reset_alt_sp();

			/* set altitude setpoint move rate */
			// 设置高度设定值移动速度
			_vel_sp(2) = _pos_sp_triplet.current.vz;

			_run_alt_control = false; /* request velocity setpoint to be used, instead of position setpoint */
									  // 需要使用的速度设定值，而不是位置设定值
		}

	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
		const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	// 将球体的中心投影到线上  归一化AB
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	/* we have triangle CDX with known CD and CX = R, find DX */
	if (sphere_r > cd_len) {
		/* have two roots, select one in A->B direction from D */
		// 得到两个根，从D中选择A-B方向的这一个
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
		res = d + ab_norm * dx_len;
		return true;

	} else {
		/* have no roots, return D */
		res = d;
		return false;
	}
}

void MulticopterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	// 自动模式被激活或者不在手动控制模式下时，复位位置设定值
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
		}

		_reset_pos_sp = true;
		_reset_alt_sp = true;

		reset_pos_sp();
		reset_alt_sp();
	}

//////////////////获取三重位置设定值////////////////// 
	//Poll position setpoint
	// 轮询位置设定值
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		//保证位置设定值有效
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	bool current_setpoint_valid = false;//初始化标志位
	bool previous_setpoint_valid = false;

	math::Vector<3> prev_sp;
	math::Vector<3> curr_sp;

	if (_pos_sp_triplet.current.valid) {
//////////////如果当前三重位置设定值合法,将当前设定值经纬度和高度转换成地坐标系的xyz值////////////  
		/* project setpoint to local frame */
		// 将设定值投影到本地坐标系
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		if (PX4_ISFINITE(curr_sp(0)) &&
		    PX4_ISFINITE(curr_sp(1)) &&
		    PX4_ISFINITE(curr_sp(2))) {
			current_setpoint_valid = true;
		}
	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp.data[0], &prev_sp.data[1]);
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			previous_setpoint_valid = true;
		}
	}

/*********************  下部分只是将设定值进行比例变换，缩小进一个区间   ******************/  
/////////////范围区间：位置误差导致的最大允许速度 为1，也就是说(0,1)之间///////////////  
	if (current_setpoint_valid) {
		/* scaled space: 1 == position error resulting max allowed speed */
		// scaled space: 1 == 位置错误导致产生最大的允许速度     比例空间？
		math::Vector<3> cruising_speed = _params.vel_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			cruising_speed(0) = _pos_sp_triplet.current.cruising_speed;
			cruising_speed(1) = _pos_sp_triplet.current.cruising_speed;
		}

		math::Vector<3> scale = _params.pos_p.edivide(cruising_speed);//水平位置误差的比例增益pos_p/自动模式下的一般水平速度cruising_speed

		/* convert current setpoint to scaled space */
		// 将当前设定值转换到比例空间
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		// 默认情况下就使用当前的设定值
		math::Vector<3> pos_sp_s = curr_sp_s;

		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) &&
		    previous_setpoint_valid) {

			/* follow "previous - current" line */
			// 沿着 "之前 -> 现在" 线路
			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of unit sphere and trajectory */
				// 单元球体与与轨迹的交点
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();
/////////////////根据飞行器位置距离当前位置设定点的距离分2种情况：小于单位半径和不小于单位半径////////////////////  
				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than unit radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					// 飞行器距航点的位置小于单位半径
					// 检测下一个航点的位置并使用它来防止飞行器经过一个航点时减速
					if (_pos_sp_triplet.next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
								       _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
								       &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							// 计算角度   之前 -> 现在 -> 下一刻
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							// cos(a)*curr_next，a表示当前路线和下一段轨迹间的夹角
							/* prev_curr_s_norm是单位向量(当前位置设定 - 前一次位置设定)，curr_next_s是另一个向量(下一次位置设定 - 当前位置设定)*/  
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;// 向量点乘

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							/* curr_pos_s是向量当前位置指向实际位置(实际位置 - 当前位置设定) 
                             * prev_curr_s_norm是前一次位置设定指向当前位置设定的单位向量(当前位置设定-前一次位置设定) 
                             * curr_pos_s_len是当前位置设定与实际位置之间长度 
                             * 所以cos_b就是pos - curr_sp - prev_sp三点连线的角度的余弦值??
                             */  
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();

								/* if curr - next distance is larger than unit radius, limit it */
								// 如果 当前 -> 下一刻  距离大于单位半径，限制到单位半径内
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								// 前馈位置设定值偏移
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {//不小于单位半径
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

					if (near) {
						/* unit sphere crosses trajectory */
						// 单位球经过了轨迹
					} else {
						/* copter is too far from trajectory */
						/* if copter is behind prev waypoint, go directly to prev waypoint */
						// 飞行器距离轨迹太远
						// 如果飞行器在上一个航点后面，直接去上一个航点
						// 角pos_sp - prev_sp - curr_sp大于90° 
						if ((pos_sp_s - prev_sp_s) * prev_curr_s < 0.0f) {
							pos_sp_s = prev_sp_s;
						}

						/* if copter is in front of curr waypoint, go directly to curr waypoint */
						// 如果飞行器在当前航点之前，直接回当前航点
						if ((pos_sp_s - curr_sp_s) * prev_curr_s > 0.0f) {
							pos_sp_s = curr_sp_s;
						}

						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}
/* 由上述程序大概就可以看出控制逻辑 
 * 先根据任务设定前一个、当前、下一个位置标定(prev_sp_s、curr_sp_s、next_sp_s)，用于控制飞行器按照此轨迹飞行 
 * pos_sp_s是实时位置设定值，用于指导飞行器具体如何一个点一个点的靠近轨迹标定值(prev_sp_s、curr_sp_s、next_sp_s) 
 * pos_s是飞行器实时位置 
 * 带了_s的都是经过大小比例缩放的，不是实际值 
 */ 

		////////////////以下部分是限速用的//////////////  
		/* move setpoint not faster than max allowed speed */
		// 设置点的移动速度小于最大允许速度
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		// 当前位置与目标位置设定值之间距离，  1=最大速度
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();

		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);
		////////////////以上部分是限速用的//////////////  

		/* update yaw setpoint if needed */
		// 需要时更新偏航姿态设定值
		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		 // 如果已经接近当前的起飞点，为防止接下来切换到定点模式，不要进行复位
		 // 这会使得起飞过程能平缓的完成
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    // 在navigator从起飞切换到下一个航点之前 需要检测飞机是否在朝着下一个航点的方向靠近
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_reset_pos_sp = false;
			_reset_alt_sp = false;

			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */
			// 否则：为防止任务中断，不要去下一个航点，停在当前位置

		} else {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
		// 没有航点，什么都不用做，设定值已经复位了
	}
}

void
MulticopterPositionControl::task_main()
{

	/*
	 * do subscriptions
	 * 主题订阅
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));


	parameters_update(true); //参数初始化，为true->强制更新

	/* initialize values of critical structs until first regular update */
	// 第一次更新前初始化关键结构体的成员值
	_arming.armed = false; // 不解锁

	/* get an initial update for all sensor and status data */
	poll_subscriptions(); // 主题数据初始化，对订阅的主题进行数据获取 check&copy

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool reset_yaw_sp = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0;

	math::Vector<3> thrust_int;
	thrust_int.zero();


	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20); //频率50Hz

		/* timed out - periodic check for _task_should_exit */
	    // 超时 - 周期性检测 _task_should_exit
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
			// 以50Hz频率接收手动输入
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);
/////////////////////对解锁状态进行判断之后复位位置和高度设置/////////////////  
		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_vel_sp_prev.zero();
			reset_int_z = true;
			reset_int_xy = true;
			reset_yaw_sp = true;
		}
////////////////固定翼模式为垂直起降控制,复位yaw和高度设置/////////////////////////
		/* reset yaw and altitude setpoint for VTOL which are in fw mode */
		// 对于固定翼模式下的VTOL，复位偏航以及高度设定值
		if (_vehicle_status.is_vtol) {
			if (!_vehicle_status.is_rotary_wing) {
				reset_yaw_sp = true;
				_reset_alt_sp = true;
			}
		}

		//Update previous arming state
		// 更新上一刻的解锁状态
		was_armed = _control_mode.flag_armed;

		update_ref(); //用位置估计的参数来更新当前位置参考点 

		/*
		 * Update velocity derivative,
		 * independent of the current flight mode
		 */ 
		//独立于当前模式更新加速度
		if (_local_pos.timestamp > 0) {

			if (PX4_ISFINITE(_local_pos.x) &&
			    PX4_ISFINITE(_local_pos.y) &&
			    PX4_ISFINITE(_local_pos.z)) {

				_pos(0) = _local_pos.x;
				_pos(1) = _local_pos.y;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {//声呐有效？
					_pos(2) = -_local_pos.dist_bottom;

				} else {
					_pos(2) = _local_pos.z;
				}
			}

			if (PX4_ISFINITE(_local_pos.vx) &&
			    PX4_ISFINITE(_local_pos.vy) &&
			    PX4_ISFINITE(_local_pos.vz)) {

				_vel(0) = _local_pos.vx;
				_vel(1) = _local_pos.vy;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_vel(2) = -_local_pos.dist_bottom_rate;

				} else {
					_vel(2) = _local_pos.vz;
				}
			}
			//获取速度的微分量，一个低通滤波器的实现
			_vel_err_d(0) = _vel_x_deriv.update(-_vel(0)); //当前速度的微分
			_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
			_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
		}


////////////以下的_control_mode.xxxxxx均来自commander.cpp///////////////////////  
		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		// 将水平和垂直位置保持标志复位用于非手动模式或者位置/高度不可控的情况
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}
/*****************位置控制逻辑的集中体现*****************/  
////////高度控制、位置控制、爬升速率(竖直方向)控制、速度(水平方向)控制任意一个使能则进入以下这段程序////////// 
		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) {

			_vel_ff.zero();

			/* by default, run position/altitude controller. the control_* functions
			 * can disable this and run velocity controllers directly in this cycle */
			// 默认情况下，运行位置/高度控制器，也可以直接运行速度控制器。control_* 函数可以禁用此项并直接在循环中运行速度控制器
			_run_pos_control = true;
			_run_alt_control = true;

/*****************第一步：产生位置/速度设定值(期望值)*****************/  
////////////选择控制源是手动、外部(offboard)、或者自动控制，产生位置/速度设定值(期望值)//////////////////  
////////////这部分的三个函数具体会在下面展开/////////////////////  
			/* select control source */
			// 选择控制源，飞行模式
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control 手动模式*/
				control_manual(dt);
				_mode_auto = false;

			} else if (_control_mode.flag_control_offboard_enabled) {
				/* offboard control 外部控制模式*/
				control_offboard(dt);
				_mode_auto = false;

			} else {
				/* AUTO  自动模式*/
				control_auto(dt);
			}

/*****************第二步：产生姿态设定值(期望值)*****************/  
////////////////////vtol不用管，它是用于固定翼的/////////////////// 
			/* weather-vane mode for vtol: disable yaw control */
			// disable_mc_yaw_control仅用于VTOL的风向标模式
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.disable_mc_yaw_control == true) {
				_att_sp.disable_mc_yaw_control = true;

			} else {
				/* reset in case of setpoint updates */
				// 万一设定值更新，先进行复位
				_att_sp.disable_mc_yaw_control = false;
			}

			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
			    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				// 空闲状态，不运行控制器并将推力置0
				R.identity();
				//将姿态设定值的R_body[9]数组复制到R矩阵中  
				memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;  
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				//　发布姿态设定值
//////////////////////此处发布的姿态设定值不是正常使用的，都被置为0了//////////////////////////  
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);
				// 句柄为空时，先公告该话题
				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} else if (_control_mode.flag_control_manual_enabled
				   && _vehicle_land_detected.landed) {
				/* don't run controller when landed */
				// 着陆时不运行控制器(所以位置和高度设定值等复位)
				_reset_pos_sp = true;
				_reset_alt_sp = true;
				_mode_auto = false;
				reset_int_z = true;
				reset_int_xy = true;

				R.identity();
				//将姿态设定值的R_body[9]数组复制到R矩阵中  
				memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				// 发布姿态设定值
//////////////////这里发布的姿态设定值是和着陆模式有关的，并不是飞行的姿态设定值///////////////  
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} 
/*****************重点：产生姿态设定值(期望值)*****************/  
/////////////////这段程序应该才是发布正常飞行状态的姿态设定值//////////////////  
/////////////////运行位置和高度控制器(否则采用已经计算出来的速度设定值)//////////////  
			else {
/****************重点(1):产生可利用的速度设定值(期望值)*******************/  
				/* run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
				// 使能情况下，运行位置&姿态控制器。否则使用之前计算得到的速度设定值
				if (_run_pos_control) {
					_vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0); //位置控制外环仅使用P控制
					_vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
				}
            /*  
             *  _run_pos_control = true;//标志位 
             *  _run_alt_control = true;//标志位 
             * 
             *  _pos(n)就是之前orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos); 
             *  _pos(0) = _local_pos.x; 
             *  _pos(1) = _local_pos.y; 
             *  _pos(2) = _local_pos.z; 
             * 
             *  _pos_sp就是之前control_manual(dt);control_offboard(dt);control_auto(dt);的输出值->目标位置
             */  
				
				// guard against any bad velocity values
				// 防止有问题的速度值造成影响
				bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
						      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
						      _pos_sp_triplet.current.velocity_valid;

				// do not go slower than the follow target velocity when position tracking is active (set to valid)
				// 当激活轨迹追踪时，飞机的速度不能比追随目标速度慢
				if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
				    velocity_valid &&
				    _pos_sp_triplet.current.position_valid) {

					math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);

					float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());

					// only override velocity set points when uav is traveling in same direction as target and vector component
					// is greater than calculated position set point velocity component
					// 唯一的速度设定值超速是在飞行器朝着与目标相同的方向飞行时，向量部分比算得的位置设定速度大
					if (cos_ratio > 0) {
						ft_vel *= (cos_ratio);
						// min speed a little faster than target vel
						// 最小速度，稍微比目标速度快
						ft_vel += ft_vel.normalized() * 1.5f;

					} else { // cos_ratio<=0
						ft_vel.zero();
					}

					_vel_sp(0) = fabs(ft_vel(0)) > fabs(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0); // 速度设定值选取相对大的值
					_vel_sp(1) = fabs(ft_vel(1)) > fabs(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);

					// track target using velocity only
					// 仅使用水平方向的速度跟随目标
				} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
					   velocity_valid) {

					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;
				}

				if (_run_alt_control) {
					_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);//高度控制外环仅使用P控制
				}

				/* make sure velocity setpoint is saturated in xy*/
				// 确保速度设定值在xy方向上饱和
				float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
							  _vel_sp(1) * _vel_sp(1));

				if (vel_norm_xy > _params.vel_max(0)) {
					/* note assumes vel_max(0) == vel_max(1) */
					// 假定x和y方向上最大速度相同
					_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
					_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
				}

				 //////////////////以下是设定垂直速度/////////////// 
				/* make sure velocity setpoint is saturated in z*/
				// 确保速度设定值在Z方向上饱和
				if (_vel_sp(2) < -1.0f * _params.vel_max_up) { // z轴向下
					_vel_sp(2) = -1.0f * _params.vel_max_up; //最大向上速度
				}

				if (_vel_sp(2) >  _params.vel_max_down) {
					_vel_sp(2) = _params.vel_max_down;		 //最大向下速度
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
				}

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
				}

				if (!_control_mode.flag_control_velocity_enabled) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
					control_vel_enabled_prev = false;
				}

				if (!_control_mode.flag_control_climb_rate_enabled) {
					_vel_sp(2) = 0.0f;
				}
				/////////////////以下是起飞的垂直速度设定///////////////// 
				/* use constant descend rate when landing, ignore altitude setpoint */
				// 降落时使用固定的下降速度，忽略高度设定值
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
					_vel_sp(2) = _params.land_speed;
				}

				/* special thrust setpoint generation for takeoff from ground */
				// 从地面起飞时特定的推力设定值
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
				    && _control_mode.flag_armed) {

					// check if we are not already in air.
					// if yes then we don't need a jumped takeoff anymore
					// 检测飞行器是否已经在空中了，如果在空中了，就不需要一个jumped takeoff了
					if (!_takeoff_jumped && !_vehicle_land_detected.landed && fabsf(_takeoff_thrust_sp) < FLT_EPSILON) {
						_takeoff_jumped = true;//进行takeoff_jump
					}

					if (!_takeoff_jumped) {
						// ramp thrust setpoint up
						// 向上的倾斜推力设定值
						if (_vel(2) > -(_params.tko_speed / 2.0f)) {
							_takeoff_thrust_sp += 0.5f * dt;
							_vel_sp.zero();
							_vel_prev.zero();

						} else {
							// copter has reached our takeoff speed. split the thrust setpoint up
							// into an integral part and into a P part
							// 飞行器已经达到了起飞速度。将向上的推力设定值分为P部分和I部分
							thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
							thrust_int(2) = -math::constrain(thrust_int(2), _params.thr_min, _params.thr_max);
							_vel_sp_prev(2) = -_params.tko_speed;
							_takeoff_jumped = true;
							reset_int_z = false;
						}
					}

					if (_takeoff_jumped) {
						_vel_sp(2) = -_params.tko_speed;
					}

				} else {
					_takeoff_jumped = false;
					_takeoff_thrust_sp = 0.0f;
				}
 				 ///////////////以上是起飞垂直速度设定///////////////////// 

				// limit total horizontal acceleration
				// 限制总的水平加速度
				math::Vector<2> acc_hor;
				acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
				acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;

				if (acc_hor.length() > _params.acc_hor_max) {
					acc_hor.normalize();
					acc_hor *= _params.acc_hor_max;//限幅
					math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
					math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
					_vel_sp(0) = vel_sp_hor(0);//修改限幅后的水平x方向速度设定 
					_vel_sp(1) = vel_sp_hor(1);//修改限幅后的水平y方向速度设定 
				}

				// limit vertical acceleration
				// 限制垂直方向的加速度
				float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;

				if (fabsf(acc_v) > 2 * _params.acc_hor_max) {
					acc_v /= fabsf(acc_v);
					_vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
				}

				_vel_sp_prev = _vel_sp;

				_global_vel_sp.vx = _vel_sp(0); // 更新全球速度设定值
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				// 发布速度设定值
				if (_global_vel_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

				} else {
					_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
				}
		/************************************************************************************************ 
         * orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet); 
         * 经过control_manual(dt);control_offboard(dt);control_auto(dt);输出pos_sp 
         * 经过上部分输出_vel_sp 
         * 发布_global_vel_sp 
         ************************************************************************************************/  

 /****************重点(2):产生可利用的推力定值(期望值)*******************/  
////////////////////爬升速率控制使能||水平速度控制使能///////////////////////  
				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {
					/* reset integrals if needed */
					// 需要时重置积分
					if (_control_mode.flag_control_climb_rate_enabled) { //爬升速率控制使能  
						if (reset_int_z) {
							reset_int_z = false;
							float i = _params.thr_min;

							if (reset_int_z_manual) {
								i = _params.thr_hover;

								if (i < _params.thr_min) {
									i = _params.thr_min;

								} else if (i > _params.thr_max) {
									i = _params.thr_max;//限幅
								}
							}

							thrust_int(2) = -i; //赋值
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {//水平速度控制使能  
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					// 速度误差
					math::Vector<3> vel_err = _vel_sp - _vel;
					/*  _vel是实际飞行器的速度 
                     *  _vel(0) = _local_pos.vx; 
                     *  _vel(1) = _local_pos.vy; 
                     *  _vel(2) = _local_pos.vz; 
                     *  struct vehicle_local_position_s         _local_pos;  
                     *  orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos); 
                     */  
                     
					// check if we have switched from a non-velocity controlled mode into a velocity controlled mode
					// if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
					// 检查是否从非速度控制模式切换到了速度控制模式
					// 如果是的，那么就要校正xy方向的速度，这样姿态设定值就是连贯的了
					if (!control_vel_enabled_prev && _control_mode.flag_control_velocity_enabled) {

						// choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
						// given by the last attitude setpoint
						// 选择xyz方向的速度设定值，这样最终的推力设定值方向就由上一个姿态设定值决定
						_vel_sp(0) = _vel(0) + (-PX4_R(_att_sp.R_body, 0,
									       2) * _att_sp.thrust - thrust_int(0) - _vel_err_d(0) * _params.vel_d(0)) / _params.vel_p(0);
						_vel_sp(1) = _vel(1) + (-PX4_R(_att_sp.R_body, 1,
									       2) * _att_sp.thrust - thrust_int(1) - _vel_err_d(1) * _params.vel_d(1)) / _params.vel_p(1);
						_vel_sp(2) = _vel(2) + (-PX4_R(_att_sp.R_body, 2,
									       2) * _att_sp.thrust - thrust_int(2) - _vel_err_d(2) * _params.vel_d(2)) / _params.vel_p(2);
						_vel_sp_prev(0) = _vel_sp(0);
						_vel_sp_prev(1) = _vel_sp(1);
						_vel_sp_prev(2) = _vel_sp(2);
						control_vel_enabled_prev = true;

						// compute updated velocity error
						// 更新速度误差
						vel_err = _vel_sp - _vel;
					}

					/* thrust vector in NED frame */
					// NED系中的推力向量
					math::Vector<3> thrust_sp;
					//推力设定值(三维)=速度差*P+速度差的差*D+积分  
        	/********************************************************* 
         	************上部分就将设定速度转变成设定推力************* 
         	*********************************************************/   
					if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
						thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

					} else {
						thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + thrust_int;
					}
					 
					if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
					    && !_takeoff_jumped && !_control_mode.flag_control_manual_enabled) {
						// for jumped takeoffs use special thrust setpoint calculated above
						// 对于jump-takeoff 使用上面计算得到的推力设定值
						thrust_sp.zero();
						thrust_sp(2) = -_takeoff_thrust_sp;
					}

					if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}

					if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(2) = 0.0f;
					}

					/* limit thrust vector and check for saturation */
					// 限制推力向量并检测是否饱和
					bool saturation_xy = false;
					bool saturation_z = false;

					/* limit min lift */
					// 限制最小升力
					float thr_min = _params.thr_min;

					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						// 在手动高度模式下，不允许向下的推力方向
						thr_min = 0.0f;
					}

					float thrust_abs = thrust_sp.length();
					float tilt_max = _params.tilt_max_air;
					float thr_max = _params.thr_max;
					/* filter vel_z over 1/8sec */
					_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);//垂直速度低通滤波  
					/* filter vel_z change over 1/8sec */
					float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;//垂直加速度低通滤波  
					_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

					/* adjust limits for landing mode */
					/***********************着陆处理************************/ 
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						// 降落时限制最大的倾斜和最小的升力
						tilt_max = _params.tilt_max_land;

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}

						/* descend stabilized, we're landing */
						// 稳定降落ing
						if (!_in_landing && !_lnd_reached_ground
						    && (float)fabs(_acc_z_lp) < 0.1f
						    && _vel_z_lp > 0.5f * _params.land_speed) {
							_in_landing = true;
						}

						/* assume ground, cut thrust */
						// 假定已着陆，切断推力
						if (_in_landing
						    && _vel_z_lp < 0.1f) {
							thr_max = 0.0f;
							_in_landing = false;
							_lnd_reached_ground = true;
						}

						/* once we assumed to have reached the ground always cut the thrust.
							Only free fall detection below can revoke this
						*/
						// 一旦假定已经着陆，便切断推力！。只有检测到飞机自由落体才能撤销这个
						if (!_in_landing && _lnd_reached_ground) {
							thr_max = 0.0f;
						}

						/* if we suddenly fall, reset landing logic and remove thrust limit */
						// 如果突然掉落，复位降落逻辑并移除推力限制
						if (_lnd_reached_ground
						    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
							// 假定自由落体加速度超过了4m/s^2或者速度大于1m/s
						    && (_acc_z_lp > 4.0f
							|| _vel_z_lp > 2.0f * _params.land_speed)) {
							thr_max = _params.thr_max;
							_in_landing = false;
							_lnd_reached_ground = false;
						}

					} else {
						_in_landing = false;
						_lnd_reached_ground = false;
					}
					/***********************着陆处理完毕************************/  
					
					/* limit min lift */
					// 限制最小升力
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

						/* limit max tilt */
						// 限制最大倾角(xy方向推力限幅)  
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							// 绝对水平推力
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								// 对于给定的垂直推力的最大水平推力 
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}
					}

					if (_control_mode.flag_control_altitude_enabled) {
						/* thrust compensation for altitude only control modes */
						// 仅用于高度控制模式的推力补偿
						float att_comp;

						if (_R(2, 2) > TILT_COS_MAX) {
							att_comp = 1.0f / _R(2, 2);

						} else if (_R(2, 2) > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
							saturation_z = true;

						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}

						thrust_sp(2) *= att_comp;
					}

					/* limit max thrust */
					// 限制最大推力
					thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */
													 // 因为可能已经改变了，所以要重新计算

					if (thrust_abs > thr_max) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > thr_max) {
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -thr_max;
								saturation_xy = true;
								saturation_z = true;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = thr_max / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}

						thrust_abs = thr_max;
					}

					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}

			/*********************重点(3)：根据推力向量计算姿态设定值(期望姿态)***********************/  
            /*********************最后使用用于控制的四元数表达的旋转矩阵(旋转矩阵就是姿态)***********************/
					/* calculate attitude setpoint from thrust vector */
					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						/*************先求出body_x、body_y、body_z*****************/  
                        ///////////body_x、body_y、body_z应该是方向余弦矩阵的三个列向量//////////////  
						math::Vector<3> body_x;
						math::Vector<3> body_y;
						math::Vector<3> body_z;

						if (thrust_abs > SIGMA) {
							body_z = -thrust_sp / thrust_abs;//body_z矩阵是推力设定值矩阵的标准化

						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);
						//y_C相当于是矩阵(-sin(偏航角),cos(偏航角),0)  
						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z;

							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}

							body_x.normalize();

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}

						/* desired body_y axis */
						body_y = body_z % body_x;

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}

						/* copy rotation matrix to attitude setpoint topic */
						memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
						_att_sp.R_valid = true;

						/* copy quaternion setpoint to attitude setpoint topic */
						math::Quaternion q_sp;
						q_sp.from_dcm(R);
						memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));

						/* calculate euler angles, for logging only, must not be used for control */
						/****************由旋转矩阵R得到姿态设置欧拉角，只是log调试用，不是给控制用的****************/  
						math::Vector<3> euler = R.to_euler();
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						R.from_euler(0.0f, 0.0f, _att_sp.yaw_body);

						/* copy rotation matrix to attitude setpoint topic */
						memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
						_att_sp.R_valid = true;

						/* copy quaternion setpoint to attitude setpoint topic */
						math::Quaternion q_sp;
						q_sp.from_dcm(R);
						memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}

					_att_sp.thrust = thrust_abs;
					 //推力向量的长度赋值给姿态推力设定值(att_sp.thrust)，这样才够各个方向力度分配  
					 
					/* save thrust setpoint for logging */
					 //用于log，方便调试
					_local_pos_sp.acc_x = thrust_sp(0) * ONE_G;
					_local_pos_sp.acc_y = thrust_sp(1) * ONE_G;
					_local_pos_sp.acc_z = thrust_sp(2) * ONE_G;

					_att_sp.timestamp = hrt_absolute_time();


				} else {
					reset_int_z = true;
				}
			}

 /*********************最后：将之前程序得到的各种信息填充_local_pos_sp结构体，并发布出去***********************/  
			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0); // 位置设定值
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			_local_pos_sp.vx = _vel_sp(0); //速度设定值
			_local_pos_sp.vy = _vel_sp(1);
			_local_pos_sp.vz = _vel_sp(2);

			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

		}
////////高度控制、位置控制、爬升速率控制、速度控制的相关程序结束//////////  
//////////////其他情况(位置控制失能)就复位各种设定值////////////  
		else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			_mode_auto = false;
			reset_int_z = true;
			reset_int_xy = true;
			control_vel_enabled_prev = false;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			/* reset yaw setpoint to current position if needed */
			// 需要时重置偏航设定值到当前的位置
			if (reset_yaw_sp) {
				reset_yaw_sp = false;
				_att_sp.yaw_body = _yaw;
			}

			/* do not move yaw while sitting on the ground */
			else if (!_vehicle_land_detected.landed &&
				 !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {

				/* we want to know the real constraint, and global overrides manual */
				const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
							   _params.global_yaw_max;
				const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

				_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
				float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
				float yaw_offs = _wrap_pi(yaw_target - _yaw);

				// If the yaw offset became too big for the system to track stop
				// shifting it, only allow if it would make the offset smaller again.
				if (fabsf(yaw_offs) < yaw_offset_max ||
				    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
				    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
					_att_sp.yaw_body = yaw_target;
				}
			}

			/* control throttle directly if no climb rate controller is active */
			if (!_control_mode.flag_control_climb_rate_enabled) {
				//手动推力的转换，以便控制器输出控制推力力度  
				float thr_val = throttle_curve(_manual.z, _params.thr_hover);
				_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

				/* enforce minimum throttle if not landed */
				// 如果没有着陆，则需要限制最小推力
				if (!_vehicle_land_detected.landed) {
					_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
				}
			}

			/* control roll and pitch directly if no aiding velocity controller is active
			 * and only if optimal recovery is not used */
			if (!_control_mode.flag_control_velocity_enabled
			    && _params.opt_recover <= 0) {
				math::Matrix<3, 3> R_sp;
				_att_sp.roll_body = _manual.y * _params.man_roll_max;
				_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

				// construct attitude setpoint rotation matrix. modify the setpoints for roll
				// and pitch such that they reflect the user's intention even if a yaw error
				// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
				// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
				// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
				// heading of the vehicle.

				// calculate our current yaw error
				float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

				// compute the vector obtained by rotating a z unit vector by the rotation
				// given by the roll and pitch commands of the user
				math::Vector<3> zB = {0, 0, 1};
				math::Matrix<3, 3> R_sp_roll_pitch;
				R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
				math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;


				// transform the vector into a new frame which is rotated around the z axis
				// by the current yaw error. this vector defines the desired tilt when we look
				// into the direction of the desired heading
				math::Matrix<3, 3> R_yaw_correction;
				R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
				z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

				// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
				// to calculate the new desired roll and pitch angles
				// R_tilt can be written as a function of the new desired roll and pitch
				// angles. we get three equations and have to solve for 2 unknowns
				float pitch_new = asinf(z_roll_pitch_sp(0));
				float roll_new = -atan2f(z_roll_pitch_sp(1), z_roll_pitch_sp(2));

				R_sp.from_euler(roll_new, pitch_new, _att_sp.yaw_body);
				memcpy(&_att_sp.R_body[0], R_sp.data, sizeof(_att_sp.R_body));

				/* copy quaternion setpoint to attitude setpoint topic */
				math::Quaternion q_sp;
				q_sp.from_dcm(R_sp);
				memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));
			}

			_att_sp.timestamp = hrt_absolute_time();

		} 
///////////////手动控制部分结束///////////////////////  
///////////////手动控制失能///////////////  
		else {
			reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}

		/* update previous velocity for velocity controller D part */
/////////////跟新前一个时刻的速度用于D部分(D应该是PID的D)////////////////////  
		_vel_prev = _vel;

		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity/accel control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app. Also do not publish
		 * if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
		 * attitude setpoints for the transition).
		 */
//////////////发布姿态设定值///////////////////  
/////////////////如果位置/速度失能而外部(offboard)使能，则不发布姿态设定值//////////////  
/////////////////因为这种情况姿态设定值是通过mavlink应用发布的//////////////  
/////////////////飞机工作于垂直起降或者做一个过渡，也不发布，因为此时由垂直起降控制部分发布/////////  
		if (!(_control_mode.flag_control_offboard_enabled &&
		      !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) {

			if (_att_sp_pub != nullptr) {
				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

			} else if (_attitude_setpoint_id) {
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		///////////////////手动控制后复位高度控制的积分(悬停油门)，以便更好的转变为手动模式////////////////  
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled
				     && !_control_mode.flag_control_climb_rate_enabled;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
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
