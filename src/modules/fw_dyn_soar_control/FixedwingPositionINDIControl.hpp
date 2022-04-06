/**
 * Implementation of a generic incremental position controller 
 * using incremental nonlinear dynamic inversion and differential flatness
 * for a fixed wing aircraft during dynamic soaring cycles.
 * The controller directly outputs actuator deflections for ailerons, elevator and rudder.
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

// use inclusion guards
#ifndef FIXEDWINGPOSITIONINDICONTROL_HPP_
#define FIXEDWINGPOSITIONINDICONTROL_HPP_

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/npfg/npfg.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/landing_slope/Landingslope.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airflow_aoa.h>
#include <uORB/topics/airflow_slip.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/uORB.h>


using namespace time_literals;


class FixedwingPositionINDIControl final : public ModuleBase<FixedwingPositionINDIControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingPositionINDIControl();
	~FixedwingPositionINDIControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t	_mavlink_log_pub{nullptr};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // Subscriptions
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};             // airspeed 
    uORB::Subscription _airflow_aoa_sub{ORB_ID(airflow_aoa)};                           // angle of attack
    uORB::Subscription _airflow_slip_sub{ORB_ID(airflow_slip)};                         // angle of sideslip
    uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};   // global position
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};     // local NED position
    uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};                 // vehicle velocity
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};         // vehicle acceleration
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};                 // vehicle attitude
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // vehicle body rates
    uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)}; // vehicle body accel
	

    // Publishers
	uORB::Publication<vehicle_attitude_setpoint_s>		_attitude_sp_pub;
	uORB::Publication<position_controller_status_s>		_pos_ctrl_status_pub{ORB_ID(position_controller_status)};			///< navigation capabilities publication

    // Message structs
	manual_control_setpoint_s	_manual_control_setpoint {};			///< r/c channel data
	position_setpoint_triplet_s	_pos_sp_triplet {};		///< triplet of mission items
	vehicle_attitude_setpoint_s	_att_sp {};			///< vehicle attitude setpoint
	vehicle_control_mode_s		_control_mode {};		///< control mode
	vehicle_local_position_s	_local_pos {};			///< vehicle local position
	vehicle_status_s		    _vehicle_status {};		///< vehicle status

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	perf_counter_t	_loop_perf;				///< loop performance counter

	float _pitch{0.0f};
	float _yaw{0.0f};
	float _yawrate{0.0f};

	matrix::Vector3f _body_acceleration{};
	matrix::Vector3f _body_velocity{};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};				///< captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter{0};				///< captures the number of times the estimator has reset the altitude state

	
	// Update our local parameter cache.
	int		parameters_update();

	// Update subscriptions
	void		airspeed_poll();
	void		control_update();
	void 		manual_control_setpoint_poll();
	void		vehicle_attitude_poll();
	void		vehicle_command_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_status_poll();
	void        wind_poll();

	void		status_publish();

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,

	)

};



#endif // FIXEDWINGPOSITIONINDICONTROL_HPP_