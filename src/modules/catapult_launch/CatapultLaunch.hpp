/****************************************************************************
 *
 * CatapultLaunch.hpp
 * Catapult launch automation for tube-launched fixed-wing aircraft.
 *
 * Approach B (PoC): independent module, active only during Takeoff /
 * Mission Takeoff nav states. Does not replace existing LaunchDetector.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <drivers/drv_hrt.h>
#include <math.h>
#include <matrix/matrix/math.hpp>

class CatapultLaunch : public ModuleBase<CatapultLaunch>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CatapultLaunch();
	~CatapultLaunch() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	void Run() override;

	/* ── State machine ── */
	enum class State : uint8_t {
		IDLE           = 0,
		ARMED_WAIT     = 1,
		LAUNCHED       = 2,
		TAIL_RELEASED  = 3,
		MOTOR_STARTED  = 4,
		MANUAL_FAILSAFE = 5,
		ABORTED        = 6,
		COMPLETE       = 7,
	};

	State _state{State::IDLE};

	/* ── Timestamps ── */
	hrt_abstime _t0{0};               // launch detection time
	hrt_abstime _tail_release_time{0}; // when tail was released
	hrt_abstime _state_entry_time{0};  // time of last state transition

	/* ── Flags ── */
	bool _tail_released{false};
	bool _motor_started{false};
	bool _mavlink_auto_start_allowed{true};
	bool _manual_fs_switch_prev{false};

	/* ── RC hold timers ── */
	hrt_abstime _mot_rc_active_since{0};
	hrt_abstime _fs_rc_active_since{0};

	/* ── Motor ramp ── */
	float _motor_ramp_start_pwm{1000.f};
	hrt_abstime _motor_ramp_start_time{0};

	/* ── uORB subscriptions ── */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	/* ── uORB publications ── */
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	/* ── Cached topic data ── */
	vehicle_status_s         _vehicle_status{};
	vehicle_local_position_s _local_pos{};
	vehicle_attitude_s       _vehicle_attitude{};
	rc_channels_s            _rc_channels{};
	vehicle_land_detected_s  _land_detected{};

	/* ── Internal helpers ── */
	void updateSubscriptions();
	void transitionTo(State new_state);
	bool isInTakeoffNavState() const;
	bool isArmed() const;
	bool isKilled() const;
	bool isFailsafe() const;
	bool checkAbortConditions();
	bool checkManualFailsafe();
	bool checkMotorAutoEnabled();

	float getSelectedAccelG() const;
	float pwmToNormalized(int pwm_us) const;
	void commandTailServos(int pwm5_us, int pwm6_us);
	void commandMotor(float normalized_value);
	void commandStabilizedMode();

	float motorRampValue(hrt_abstime now) const;

	/* ── Parameters ── */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CAT_EN>)             _param_cat_en,
		(ParamInt<px4::params::CAT_TRIG_MODE>)      _param_cat_trig_mode,
		(ParamFloat<px4::params::CAT_ACC_THR_G>)    _param_cat_acc_thr_g,
		(ParamInt<px4::params::CAT_ACC_AXIS>)       _param_cat_acc_axis,
		(ParamInt<px4::params::CAT_ACC_HOLD_MS>)    _param_cat_acc_hold_ms,
		(ParamFloat<px4::params::CAT_TAIL_DLY>)     _param_cat_tail_dly,
		(ParamFloat<px4::params::CAT_MOT_DLY>)      _param_cat_mot_dly,
		(ParamInt<px4::params::CAT_MOT_AUTO>)       _param_cat_mot_auto,
		(ParamInt<px4::params::CAT_MOT_RC_CH>)      _param_cat_mot_rc_ch,
		(ParamFloat<px4::params::CAT_MOT_RC_THR>)   _param_cat_mot_rc_thr,
		(ParamInt<px4::params::CAT_MOT_RC_HOLD>)    _param_cat_mot_rc_hold,
		(ParamInt<px4::params::CAT_MAV_EN>)         _param_cat_mav_en,
		(ParamInt<px4::params::CAT_QGC_EN>)         _param_cat_qgc_en,
		(ParamInt<px4::params::CAT_TAIL5_LOCK>)     _param_cat_tail5_lock,
		(ParamInt<px4::params::CAT_TAIL5_REL>)      _param_cat_tail5_rel,
		(ParamInt<px4::params::CAT_TAIL6_LOCK>)     _param_cat_tail6_lock,
		(ParamInt<px4::params::CAT_TAIL6_REL>)      _param_cat_tail6_rel,
		(ParamInt<px4::params::CAT_MOT_STOP_PWM>)   _param_cat_mot_stop_pwm,
		(ParamInt<px4::params::CAT_MOT_STRT_PWM>)   _param_cat_mot_strt_pwm,
		(ParamFloat<px4::params::CAT_MOT_RAMP>)     _param_cat_mot_ramp,
		(ParamInt<px4::params::CAT_MOT_REQ_TAIL>)   _param_cat_mot_req_tail,
		(ParamInt<px4::params::CAT_ABORT_FS>)       _param_cat_abort_fs,
		(ParamInt<px4::params::CAT_FS_RC_CH>)       _param_cat_fs_rc_ch,
		(ParamFloat<px4::params::CAT_FS_RC_THR>)    _param_cat_fs_rc_thr,
		(ParamInt<px4::params::CAT_FS_RC_HOLD>)     _param_cat_fs_rc_hold,
		(ParamInt<px4::params::CAT_FS_TO_STAB>)     _param_cat_fs_to_stab,
		(ParamInt<px4::params::CAT_FS_MOT_EN>)      _param_cat_fs_mot_en
	)
};
