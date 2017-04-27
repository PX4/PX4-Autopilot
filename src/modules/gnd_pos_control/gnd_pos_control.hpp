#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <cfloat>

#include <arch/board/board.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include <geo/geo.h>
#include <launchdetection/LaunchDetector.h>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/pid/pid.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

using namespace launchdetection;

class GroundRoverPositionControl
{
public:
	/**
	 * Constructor
	 */
	GroundRoverPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~GroundRoverPositionControl();

	// prevent copying
	GroundRoverPositionControl(const GroundRoverPositionControl &) = delete;
	GroundRoverPositionControl operator=(const GroundRoverPositionControl &other) = delete;

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	orb_advert_t	_mavlink_log_pub;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_ctrl_state_sub;			/**< control state subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_command_sub;		/**< vehicle command subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_control_sub;		/**< notification of manual control updates */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_gnd_pos_ctrl_status_pub;		/**< navigation capabilities publication */

	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;			/**< control state */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct fw_pos_ctrl_status_s			_gnd_pos_ctrl_status;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_command_s			_vehicle_command;		/**< vehicle commands */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;		/**< vehicle land detected */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _control_position_last_called; 	/**<last call of control_position  */

	/* Takeoff launch detection and runway */
	LaunchDetectionResult _launch_detection_state;

	/* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	 the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	PID_t _speed_ctrl;

	/* throttle and airspeed states */
	float _speed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_received;		///< last time airspeed was received. Used to detect timeouts.
		
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude
	float _roll;
	float _pitch;
	float _yaw;
	bool _reinitialize_tecs;			///< indicates if the TECS states should be reinitialized (used for VTOL)
	bool _is_tecs_running;
	hrt_abstime _last_tecs_update;
	float _asp_after_transition;
	bool _was_in_transition;

	// estimator reset counters
	uint8_t _pos_reset_counter;		// captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter;		// captures the number of times the estimator has reset the altitude state

	ECL_L1_Pos_Controller				_gnd_control;
	TECS						_tecs;
	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current;			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	struct {
		float l1_period;
		float l1_damping;
		float l1_distance;

		float time_const;
		float time_const_throt;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float heightrate_p;
		float heightrate_ff;
		float speedrate_p;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
		int airspeed_mode;
		int speed_control_mode;
		float speed_p;
		float speed_i;
		float speed_d;
		float speed_imax;
		float speed_throttle_airspeed_scaler;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;

	} _parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;
		param_t l1_distance;

		param_t time_const;
		param_t time_const_throt;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t heightrate_p;
		param_t heightrate_ff;
		param_t speedrate_p;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
		param_t airspeed_mode;

		param_t speed_control_mode;
		param_t speed_p;
		param_t speed_i;
		param_t speed_d;
		param_t speed_imax;
		param_t speed_throttle_airspeed_scaler;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;

	} _parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for new in vehicle commands
	 */
	void		vehicle_command_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for changes in vehicle land detected.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for manual setpoint updates.
	 */
	bool		vehicle_manual_control_setpoint_poll();

	/**
	 * Check for changes in control state.
	 */
	void		control_state_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		gnd_pos_ctrl_status_publish();

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float		get_tecs_pitch();
	float		get_tecs_thrust();

	float		calculate_target_speed(float airspeed_demand);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Call TECS : a wrapper function to call the TECS implementation
	 */
	void tecs_update_throttle(float alt_sp, float v_sp, float eas2tas,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max, float throttle_cruise,
					float climbout_pitch_min_rad,
					float altitude,
					const math::Vector<3> &ground_speed,
					unsigned mode = tecs_status_s::TECS_MODE_NORMAL);



};