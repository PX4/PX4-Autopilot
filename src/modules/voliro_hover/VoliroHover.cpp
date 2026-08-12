#include "VoliroHover.hpp"

#include <cmath>
#include <cstring>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor VoliroHover::desc{task_spawn, custom_command, print_usage};

VoliroHover::VoliroHover() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_gl40_enable_handle = param_find("GL40_EN");
	updateConfiguration();
}

VoliroHover::~VoliroHover()
{
	perf_free(_loop_perf);
}

bool VoliroHover::init()
{
	if (!_param_enable.get()) {
		PX4_ERR("VHOV_EN is disabled");
		return false;
	}

	if (!updateConfiguration()) {
		PX4_ERR("invalid hover trajectory configuration");
		return false;
	}

	ScheduleOnInterval(20_ms); // 50 Hz reference stream and mode supervision
	requestRegistration(hrt_absolute_time());
	return true;
}

bool VoliroHover::updateConfiguration()
{
	updateParams();
	return _trajectory.configure(_param_height.get(), _param_launch_acceleration.get(),
		       _param_touchdown_acceleration.get(),
		       _param_takeoff_duration.get(), _param_land_duration.get());
}

void VoliroHover::updateInputs()
{
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_odometry_sub.update(&_vehicle_odometry);
	_land_detected_sub.update(&_land_detected);

	if (_tilt_feedback_sub.update(&_tilt_feedback)) {
		_typed_feedback_seen = true;
	}

#ifdef __PX4_POSIX
	// MuJoCo uses the allocator's legacy MAVLink DEBUG_FLOAT_ARRAY feedback
	// bridge. Hardware must pass the typed GL40II feedback path above.
	if (!_typed_feedback_seen) {
		debug_array_s feedback{};

		while (_debug_array_sub.update(&feedback)) {
			if (feedback.id != SIM_TILT_FEEDBACK_ID
			    || strncmp(feedback.name, "VOLA_TILT", sizeof(feedback.name)) != 0) {
				continue;
			}

			bool finite = true;

			for (int rotor = 0; rotor < 6; ++rotor) {
				finite &= PX4_ISFINITE(feedback.data[rotor]);
				finite &= PX4_ISFINITE(feedback.data[6 + rotor]);
			}

			if (finite) {
				_tilt_feedback.timestamp = feedback.timestamp;
				_tilt_feedback.timestamp_sample = feedback.timestamp;
				_tilt_feedback.active_mask = ALL_TILTS_MASK;
				_tilt_feedback.valid_mask = ALL_TILTS_MASK;
				_tilt_feedback.fault_mask = 0;
				_tilt_feedback.stale_mask = 0;

				for (int rotor = 0; rotor < 6; ++rotor) {
					_tilt_feedback.angle[rotor] = feedback.data[rotor];
					_tilt_feedback.angular_velocity[rotor] = feedback.data[6 + rotor];
				}
			}
		}
	}
#endif
}

void VoliroHover::requestRegistration(hrt_abstime now)
{
	register_ext_component_request_s request{};
	request.timestamp = now;
	strncpy(request.name, "VOLIRO HOVER", sizeof(request.name) - 1);
	request.request_id = EXT_COMPONENT_REQUEST_ID;
	request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
	request.register_arming_check = true;
	request.register_mode = true;
	request.request_offboard_setpoints = false;
	_register_request_pub.publish(request);
	_last_registration_request = now;
}

void VoliroHover::publishModeConfiguration(hrt_abstime now)
{
	vehicle_control_mode_s config{};
	config.timestamp = now;
	config.source_id = _mode_id;
	config.flag_multicopter_position_control_enabled = false;
	config.flag_control_manual_enabled = false;
	config.flag_control_auto_enabled = false;
	config.flag_control_offboard_enabled = false;
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = false;
	config.flag_control_altitude_enabled = false;
	config.flag_control_climb_rate_enabled = false;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = false;
	config.flag_control_allocation_enabled = true;
	config.flag_control_termination_enabled = true;
	_config_control_setpoints_pub.publish(config);
}

void VoliroHover::updateRegistration(hrt_abstime now)
{
	register_ext_component_reply_s reply{};

	while (_register_reply_sub.update(&reply)) {
		if (reply.request_id != EXT_COMPONENT_REQUEST_ID) {
			continue;
		}

		if (!reply.success || reply.mode_id < 0 || reply.arming_check_id < 0) {
			PX4_ERR("VOLIRO HOVER mode registration failed");
			continue;
		}

		_mode_id = reply.mode_id;
		_arming_check_id = reply.arming_check_id;
		_registration_state = RegistrationState::Registered;
		PX4_INFO("VOLIRO HOVER registered: mode_id=%d arming_check_id=%d", _mode_id, _arming_check_id);
	}

	if (_registration_state == RegistrationState::Unregistered
	    && (_last_registration_request == 0 || now - _last_registration_request >= 1_s)) {
		requestRegistration(now);
	}

	if (_registration_state == RegistrationState::Registered) {
		publishModeConfiguration(now);
		_registration_state = RegistrationState::Configured;
		PX4_INFO("VOLIRO HOVER mode configuration sent");
	}
}

void VoliroHover::unregisterMode()
{
	if (_registration_state == RegistrationState::Unregistered) {
		return;
	}

	unregister_ext_component_s message{};
	message.timestamp = hrt_absolute_time();
	strncpy(message.name, "VOLIRO HOVER", sizeof(message.name) - 1);
	message.arming_check_id = _arming_check_id;
	message.mode_id = _mode_id;
	message.mode_executor_id = -1;
	_unregister_pub.publish(message);
}

bool VoliroHover::armed() const
{
	return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
}

bool VoliroHover::odometryValid(hrt_abstime now) const
{
	if (_vehicle_odometry.timestamp == 0 || now < _vehicle_odometry.timestamp
	    || now - _vehicle_odometry.timestamp > static_cast<hrt_abstime>(_param_odometry_timeout.get() * 1e6f)
	    || _vehicle_odometry.pose_frame != vehicle_odometry_s::POSE_FRAME_NED) {
		return false;
	}

	const Quatf attitude{_vehicle_odometry.q};
	const bool velocity_frame_valid = _vehicle_odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_NED
					  || _vehicle_odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
	return Vector3f{_vehicle_odometry.position}.isAllFinite()
	       && Vector3f{_vehicle_odometry.velocity}.isAllFinite()
	       && Vector3f{_vehicle_odometry.angular_velocity}.isAllFinite()
	       && attitude.isAllFinite() && attitude.norm() > 1e-6f && velocity_frame_valid;
}

bool VoliroHover::tiltCentered() const
{
	for (int rotor = 0; rotor < 6; ++rotor) {
		if (!PX4_ISFINITE(_tilt_feedback.angle[rotor])
		    || fabsf(_tilt_feedback.angle[rotor]) > _param_zero_tolerance.get()) {
			return false;
		}
	}

	return true;
}

bool VoliroHover::gl40MotionEnabled() const
{
	if (_gl40_enable_handle == PARAM_INVALID) {
#ifdef __PX4_POSIX
		// SITL receives modeled tilt feedback from the MuJoCo bridge and does
		// not instantiate the hardware-only raw-CAN driver parameters.
		return true;
#else
		return false;
#endif
	}

	int32_t enabled = 0;
	return param_get(_gl40_enable_handle, &enabled) == PX4_OK && enabled == 1;
}

uint8_t VoliroHover::readinessFailure(hrt_abstime now, bool already_active) const
{
	if (!_param_enable.get() || !_param_allocator_enable.get()
	    || !_param_controller_enable.get() || !gl40MotionEnabled()) {
		return voliro_hover_status_s::FAILURE_DISABLED;
	}

	if (_registration_state != RegistrationState::Configured) {
		return voliro_hover_status_s::FAILURE_NOT_REGISTERED;
	}

	if (!odometryValid(now)) {
		return voliro_hover_status_s::FAILURE_ODOMETRY;
	}

	const bool feedback_fresh = _tilt_feedback.timestamp != 0 && now >= _tilt_feedback.timestamp
				    && now - _tilt_feedback.timestamp <= static_cast<hrt_abstime>(_param_feedback_timeout.get() * 1e6f);
	const bool masks_healthy = (_tilt_feedback.active_mask & ALL_TILTS_MASK) == ALL_TILTS_MASK
				   && (_tilt_feedback.valid_mask & ALL_TILTS_MASK) == ALL_TILTS_MASK
				   && (_tilt_feedback.fault_mask & ALL_TILTS_MASK) == 0
				   && (_tilt_feedback.stale_mask & ALL_TILTS_MASK) == 0;

	if (!feedback_fresh || !masks_healthy) {
		return voliro_hover_status_s::FAILURE_TILT_FEEDBACK;
	}

	if (!already_active) {
		if (!tiltCentered()) {
			return voliro_hover_status_s::FAILURE_TILT_NOT_CENTERED;
		}

		const bool land_state_fresh = _land_detected.timestamp != 0 && now >= _land_detected.timestamp
					      && now - _land_detected.timestamp <= 1_s;

		if (!land_state_fresh || !_land_detected.landed) {
			return voliro_hover_status_s::FAILURE_NOT_LANDED;
		}
	}

	return voliro_hover_status_s::FAILURE_NONE;
}

void VoliroHover::updateArmingCheck()
{
	if (_registration_state != RegistrationState::Configured || !_arming_check_request_sub.updated()) {
		return;
	}

	arming_check_request_s request{};
	_arming_check_request_sub.copy(&request);
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request.request_id;
	reply.registration_id = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events = 0;
	reply.can_arm_and_run = _can_arm_and_run;
	reply.mode_req_angular_velocity = true;
	reply.mode_req_attitude = true;
	reply.mode_req_local_alt = true;
	reply.mode_req_local_position = true;
#ifdef __PX4_POSIX
	reply.mode_req_manual_control = false;
#else
	// Hardware flight keeps RC loss and the assigned kill/disarm control in
	// Commander's normal safety chain. SITL has no physical RC receiver.
	reply.mode_req_manual_control = true;
#endif
	_arming_check_reply_pub.publish(reply);
}

void VoliroHover::updateModeState(hrt_abstime now)
{
	_mode_selected = _registration_state == RegistrationState::Configured
			 && _mode_id >= 0 && _vehicle_status.nav_state == static_cast<uint8_t>(_mode_id);

	if (_mode_selected && !_mode_selected_previous) {
		// Commander deliberately treats a configuration published before mode
		// activation as stale. Refresh it here so the stock multicopter
		// position/attitude/rate controllers relinquish the shared setpoints.
		publishModeConfiguration(now);

		if (readinessFailure(now, false) == voliro_hover_status_s::FAILURE_NONE) {
			const Vector3f position{_vehicle_odometry.position};
			const float yaw = Eulerf{Quatf{_vehicle_odometry.q}}.psi();
			_trajectory.reset(position, yaw);
			PX4_INFO("mode active: ground NED %.2f %.2f %.2f, yaw %.1f deg",
				 (double)position(0), (double)position(1), (double)position(2), (double)(yaw * 180.f / M_PI_F));

		} else {
			PX4_ERR("mode activated while not ready: %s", failureName(readinessFailure(now, false)));
		}
	}

	if (_mode_selected && _trajectory.initialized() && _was_armed && !armed()
	    && odometryValid(now) && _land_detected.landed) {
		const Vector3f position{_vehicle_odometry.position};
		const float yaw = Eulerf{Quatf{_vehicle_odometry.q}}.psi();
		_trajectory.reset(position, yaw);
		PX4_INFO("disarmed: recaptured ground reference");
	}

	_mode_selected_previous = _mode_selected;
	_was_armed = armed();
}

void VoliroHover::processCommands(hrt_abstime now)
{
	const uint8_t commands = _pending_commands.fetch_and(CommandNone);

	if (commands & CommandTakeoff) {
		if (!_mode_selected || !_trajectory.initialized() || !armed()) {
			PX4_ERR("takeoff rejected: select VOLIRO HOVER and arm first");

		} else if (readinessFailure(now, true) != voliro_hover_status_s::FAILURE_NONE) {
			// The Ready phase was only initialized after landed and centered
			// checks passed. Once armed, hover preload can momentarily clear the
			// land detector before this explicit command reaches the work queue.
			PX4_ERR("takeoff rejected: %s", failureName(readinessFailure(now, true)));

		} else if (_trajectory.beginLaunch()) {
			_launch_started = now;
			PX4_WARN("launch started: %.2f m/s^2 upward, %.1f s timeout",
				 (double)_param_launch_acceleration.get(), (double)_param_launch_timeout.get());

		} else {
			PX4_ERR("takeoff rejected in phase %s", phaseName());
		}
	}

	if (commands & CommandLand) {
		if (!_mode_selected || !_trajectory.initialized() || !armed() || !odometryValid(now)) {
			PX4_ERR("land rejected: active armed mode with valid odometry required");

		} else if (_trajectory.startLand(now, _vehicle_odometry.position[2])) {
			PX4_WARN("landing started: %.1f s", (double)_param_land_duration.get());

		} else {
			PX4_ERR("land rejected in phase %s", phaseName());
		}
	}
}

void VoliroHover::advanceLaunch(hrt_abstime now)
{
	if (_trajectory.phase() != VoliroHoverTrajectory::Phase::Launch) {
		return;
	}

	if (!_land_detected.landed) {
		if (_trajectory.startTakeoff(now, _vehicle_odometry.position[2])) {
			PX4_WARN("liftoff detected: climb to %.2f m in %.1f s",
				 (double)_param_height.get(), (double)_param_takeoff_duration.get());
		}

		return;
	}

	if (_launch_started != 0
	    && now - _launch_started >= static_cast<hrt_abstime>(_param_launch_timeout.get() * 1e6f)) {
		const Vector3f position{_vehicle_odometry.position};
		const float yaw = Eulerf{Quatf{_vehicle_odometry.q}}.psi();
		_trajectory.reset(position, yaw);
		_launch_started = 0;
		PX4_ERR("launch timeout: no liftoff, returned to ground hold");
	}
}

void VoliroHover::publishTrajectory(hrt_abstime now)
{
	if (!_mode_selected || !_trajectory.initialized()) {
		return;
	}

	_last_setpoint = _trajectory.update(now);
	trajectory_setpoint6dof_s message{};
	message.timestamp = now;
	_last_setpoint.position_ned.copyTo(message.position);
	_last_setpoint.velocity_ned.copyTo(message.velocity);
	_last_setpoint.acceleration_ned.copyTo(message.acceleration);
	matrix::Vector3f{}.copyTo(message.jerk);
	_last_setpoint.attitude_ned_frd.copyTo(message.quaternion);
	_last_setpoint.angular_velocity_frd.copyTo(message.angular_velocity);
	_trajectory_setpoint_pub.publish(message);
}

void VoliroHover::publishStatus(hrt_abstime now)
{
	voliro_hover_status_s status{};
	status.timestamp = now;
	status.target_height = _param_height.get();
	status.mode_id = _mode_id;
	status.phase = _mode_selected && _trajectory.initialized()
		       ? static_cast<uint8_t>(_trajectory.phase()) : voliro_hover_status_s::PHASE_INACTIVE;
	status.failure_reason = _failure_reason;
	status.registered = _registration_state == RegistrationState::Configured;
	status.active = _mode_selected && _trajectory.initialized();
	status.can_arm_and_run = _can_arm_and_run;

	if (status.active) {
		_last_setpoint.position_ned.copyTo(status.position_setpoint);

	} else {
		status.position_setpoint[0] = NAN;
		status.position_setpoint[1] = NAN;
		status.position_setpoint[2] = NAN;
	}

	_status_pub.publish(status);
}

void VoliroHover::Run()
{
	if (should_exit()) {
		unregisterMode();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);
	const hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);

		if (!updateConfiguration()) {
			PX4_ERR("hover trajectory parameter update rejected");
		}
	}

	updateInputs();
	updateRegistration(now);
	updateModeState(now);
	_failure_reason = readinessFailure(now, _mode_selected);
	_can_arm_and_run = _failure_reason == voliro_hover_status_s::FAILURE_NONE;
	updateArmingCheck();
	processCommands(now);
	advanceLaunch(now);
	publishTrajectory(now);
	publishStatus(now);
	perf_end(_loop_perf);
}

const char *VoliroHover::phaseName() const
{
	switch (_trajectory.phase()) {
	case VoliroHoverTrajectory::Phase::Inactive: return "inactive";
	case VoliroHoverTrajectory::Phase::Ready: return "ready";
	case VoliroHoverTrajectory::Phase::Launch: return "launch";
	case VoliroHoverTrajectory::Phase::Takeoff: return "takeoff";
	case VoliroHoverTrajectory::Phase::Hold: return "hold";
	case VoliroHoverTrajectory::Phase::Land: return "land";
	case VoliroHoverTrajectory::Phase::Landed: return "landed";
	}

	return "unknown";
}

const char *VoliroHover::failureName(uint8_t failure) const
{
	switch (failure) {
	case voliro_hover_status_s::FAILURE_NONE: return "none";
	case voliro_hover_status_s::FAILURE_DISABLED: return "enable parameters";
	case voliro_hover_status_s::FAILURE_ODOMETRY: return "odometry stale or invalid";
	case voliro_hover_status_s::FAILURE_TILT_FEEDBACK: return "tilt feedback unhealthy";
	case voliro_hover_status_s::FAILURE_TILT_NOT_CENTERED: return "tilts not centered";
	case voliro_hover_status_s::FAILURE_NOT_LANDED: return "vehicle not landed";
	case voliro_hover_status_s::FAILURE_NOT_REGISTERED: return "mode not registered";
	}

	return "unknown";
}

int VoliroHover::task_spawn(int argc, char *argv[])
{
	VoliroHover *instance = new VoliroHover();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int VoliroHover::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("command required");
	}

	VoliroHover *instance = get_instance<VoliroHover>(desc);

	if (instance == nullptr) {
		PX4_ERR("voliro_hover is not running");
		return PX4_ERROR;
	}

	if (strcmp(argv[0], "takeoff") == 0) {
		instance->_pending_commands.fetch_or(CommandTakeoff);
		PX4_INFO("takeoff request queued");
		return PX4_OK;
	}

	if (strcmp(argv[0], "land") == 0) {
		instance->_pending_commands.fetch_or(CommandLand);
		PX4_INFO("land request queued");
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int VoliroHover::print_status()
{
	PX4_INFO("registration=%s mode_id=%d selected=%s armed=%s",
		 _registration_state == RegistrationState::Configured ? "configured" : "pending",
		 _mode_id, _mode_selected ? "yes" : "no", armed() ? "yes" : "no");
	PX4_INFO("phase=%s can_arm_and_run=%s failure=%s", phaseName(),
		 _can_arm_and_run ? "yes" : "no", failureName(_failure_reason));
	PX4_INFO("target: %.2f m, launch %.2f, touchdown %.2f m/s^2, takeoff %.1f s, land %.1f s, zero tolerance %.1f deg",
		 (double)_param_height.get(), (double)_param_launch_acceleration.get(),
		 (double)_param_touchdown_acceleration.get(), (double)_param_takeoff_duration.get(),
		 (double)_param_land_duration.get(),
		 (double)(_param_zero_tolerance.get() * 180.f / M_PI_F));
	perf_print_counter(_loop_perf);
	return PX4_OK;
}

int VoliroHover::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Fully onboard Voliro hover mode. It captures the current NED horizontal
position and yaw on mode entry, and publishes minimum-jerk takeoff/landing
trajectory_setpoint6dof references for voliro_control. Arming alone never
starts takeoff.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("voliro_hover", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("takeoff", "Begin the configured onboard takeoff while active and armed");
	PRINT_MODULE_USAGE_COMMAND_DESCR("land", "Begin the configured onboard landing while active and armed");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int voliro_hover_main(int argc, char *argv[])
{
	return ModuleBase::main(VoliroHover::desc, argc, argv);
}
