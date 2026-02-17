#include "mc_raptor.hpp"
#undef OK

#include <rl_tools/inference/applications/l2f/operations_generic.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <sys/stat.h>

ModuleBase::Descriptor Raptor::desc{task_spawn, custom_command, print_usage};

Raptor::Raptor(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// node state
	timestamp_last_angular_velocity_set = false;
	timestamp_last_local_position_set = false;
	timestamp_last_attitude_set = false;
	timestamp_last_trajectory_setpoint_set = false;
	timestamp_last_vehicle_status_set = false;
	previous_trajectory_setpoint_stale = false;
	previous_active = false;
	timeout_message_sent = false;
	timestamp_last_policy_frequency_check_set = false;
	last_intermediate_status_set = false;
	last_native_status_set = false;
	policy_frequency_check_counter = 0;
	flightmode_state = FlightModeState::UNREGISTERED;
	can_arm = false;
	trajectory_setpoint_dt_index = 0;
	trajectory_setpoint_dts_full = false;
	trajectory_setpoint_invalid_count = 0;
	trajectory_setpoint_dt_max_since_reset = 0;
	internal_reference_activation_position[0] = 0.0f;
	internal_reference_activation_position[1] = 0.0f;
	internal_reference_activation_position[2] = 0.0f;
	internal_reference_params_changed = false;

	_actuator_motors_pub.advertise();
	_tune_control_pub.advertise();
}
void Raptor::reset()
{

	trajectory_setpoint_dt_index = 0;
	trajectory_setpoint_dts_full = false;
	trajectory_setpoint_invalid_count = 0;
	trajectory_setpoint_dt_max_since_reset = 0;
	timestamp_last_trajectory_setpoint_set = false;


	for (TI action_i = 0; action_i < EXECUTOR_SPEC::OUTPUT_DIM; action_i++) {
		this->previous_action[action_i] = RESET_PREVIOUS_ACTION_VALUE;
	}

	rlt::reset(device, executor, policy, rng);
}

Raptor::~Raptor()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

#ifdef MC_RAPTOR_EMBED_POLICY
bool Raptor::test_policy()
{
#else
bool Raptor::test_policy(FILE *f, TI input_offset, TI output_offset)
{
#endif
	using namespace rl_tools::inference::applications::l2f;
#ifndef RL_TOOLS_DISABLE_TEST
	// This tests the policy using a known input output pair that has been saved into the policy checkpoint to verify that it has been loaded correctly
	using POLICY = EXECUTOR_CONFIG::POLICY_TEST;
	POLICY::template Buffer<false> buffers_test;
	POLICY::State<false> policy_state_test;
	rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI, rl_tools::tensor::Shape<TI, 1, POLICY::OUTPUT_SHAPE::LAST>, false>>
			test_output;
	rl_tools::Mode<rl_tools::mode::Evaluation<>> mode;
	using EXAMPLE_INPUT_SPEC = MC_RAPTOR_EXAMPLE_NAMESPACE::input::SPEC;
	using EXAMPLE_OUTPUT_SPEC = MC_RAPTOR_EXAMPLE_NAMESPACE::output::SPEC;
	float acc = 0;
	uint64_t num_values = 0;
	rl_tools::inference::applications::l2f::Action<EXECUTOR_SPEC> action;

	for (TI batch_i = 0; batch_i < EXECUTOR_CONFIG::TEST_BATCH_SIZE_ACTUAL; batch_i++) {
		rl_tools::reset(device, policy, policy_state_test, rng);

		for (TI step_i = 0; step_i < EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL; step_i++) {
#ifdef MC_RAPTOR_EMBED_POLICY
			const auto step_input = rl_tools::view(device, MC_RAPTOR_EXAMPLE_NAMESPACE::input::container, step_i);
			const auto batch_input = rl_tools::view_range(device, step_input, batch_i, rlt::tensor::ViewSpec<0, 1> {});
			const auto step_output_target = rl_tools::view(device, MC_RAPTOR_EXAMPLE_NAMESPACE::output::container, step_i);
			const auto batch_output_target = rl_tools::view_range(device, step_output_target, batch_i, rlt::tensor::ViewSpec<0, 1> {});
#else
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI, rl_tools::tensor::Shape<TI, 1, EXAMPLE_INPUT_SPEC::SHAPE::LAST>, false>>
					batch_input;
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI, rl_tools::tensor::Shape<TI, 1, EXAMPLE_OUTPUT_SPEC::SHAPE::LAST>, false>>
					batch_output_target;
			fseek(f, input_offset + (step_i * EXAMPLE_INPUT_SPEC::STRIDE::FIRST + batch_i * EXAMPLE_INPUT_SPEC::STRIDE::template GET<1>)*sizeof(
				      EXAMPLE_INPUT_SPEC::T), SEEK_SET);
			fread(batch_input._data, sizeof(EXAMPLE_INPUT_SPEC::T), EXAMPLE_INPUT_SPEC::SHAPE::LAST, f);
			fseek(f, output_offset + (step_i * EXAMPLE_OUTPUT_SPEC::STRIDE::FIRST + batch_i * EXAMPLE_OUTPUT_SPEC::STRIDE::template GET<1>)*sizeof(
				      EXAMPLE_OUTPUT_SPEC::T), SEEK_SET);
			fread(batch_output_target._data, sizeof(EXAMPLE_OUTPUT_SPEC::T), EXAMPLE_OUTPUT_SPEC::SHAPE::LAST, f);
#endif
			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, batch_input), "input is nan");
			rl_tools::evaluate_step(device, policy, batch_input, policy_state_test, test_output, buffers_test, rng, mode);
			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, test_output), "output is nan");

			for (TI action_i = 0; action_i < EXECUTOR_CONFIG::OUTPUT_DIM; action_i++) {
				acc += rl_tools::math::abs(device.math, rl_tools::get(device, test_output, 0, action_i) - rl_tools::get(device, batch_output_target, 0,
							   action_i));
				num_values += 1;
				rl_tools::utils::assert_exit(device, !rl_tools::math::is_nan(device.math, acc), "output is nan");

				if (batch_i == 0 && step_i == EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL - 1) {
					action.action[action_i] = rl_tools::get(device, test_output, 0, action_i);
				}
			}
		}
	}

	float abs_diff = acc / num_values;
	PX4_INFO("Checkpoint test diff: %f", (double)abs_diff);

	for (TI output_i = 0; output_i < EXECUTOR_CONFIG::OUTPUT_DIM; output_i++) {
		PX4_INFO("output[%d]: %f", (int)output_i, (double)action.action[output_i]);
	}

	constexpr float EPSILON = 1e-5;

	bool healthy = abs_diff < EPSILON;

	if (!healthy) {
		PX4_ERR("Checkpoint test failed with diff %.10f", (double)abs_diff);
		return false;

	} else {
		PX4_INFO("Checkpoint test passed with diff %.10f", (double)abs_diff);
		return true;
	}

#else
	return 0;
#endif
}

bool Raptor::init()
{
	this->init_time = hrt_absolute_time();

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity_sub callback registration failed");
		return false;
	}

#ifndef MC_RAPTOR_EMBED_POLICY
	const char *path = PX4_STORAGEDIR "/raptor/policy.tar";

	struct stat st;
	bool file_exists = (stat(path, &st) == 0);

	if (file_exists) {
		PX4_INFO("Policy checkpoint %s exists", path);
		FILE *f = fopen(path, "rb");

		if (!f) {
			PX4_ERR("Failed to open %s: %s", path, strerror(errno));
			return false;
		}

		if (fseek(f, 0, SEEK_END) != 0) {
			PX4_ERR("fseek failed: %s", strerror(errno));
			fclose(f);
			return false;
		}

		long size = ftell(f);

		if (size < 0) {
			PX4_ERR("ftell failed: %s", strerror(errno));
			fclose(f);
			return false;

		} else {
			rewind(f);
			bool successfully_loaded = false;
			using SPEC = rlt::persist::backends::tar::ReaderGroupSpecification<TI, rlt::persist::backends::tar::PosixFileData<TI>>;

			rlt::persist::backends::tar::ReaderGroup<SPEC> reader_group;
			reader_group.data.f = f;
			reader_group.data.size = size;
			auto actor_group = rlt::get_group(device, reader_group, "actor");
			successfully_loaded = rlt::load(device, policy, actor_group);
			constexpr TI METADATA_BUFFER_SIZE = 256;
			char metadata_buffer[METADATA_BUFFER_SIZE];
			TI read_size = 0;
			rlt::persist::backends::tar::get(device, reader_group.data, "actor/meta", metadata_buffer, METADATA_BUFFER_SIZE, read_size);
			TI checkpoint_name_position = 0;
			TI checkpoint_name_len = 0;

			if (rlt::persist::backends::tar::seek_in_metadata(device, metadata_buffer, METADATA_BUFFER_SIZE, "checkpoint_name",
					checkpoint_name_position, checkpoint_name_len)) {
				strncpy(checkpoint_name, metadata_buffer + checkpoint_name_position, CHECKPOINT_NAME_LENGTH);
				checkpoint_name[checkpoint_name_len < CHECKPOINT_NAME_LENGTH ? checkpoint_name_len : CHECKPOINT_NAME_LENGTH - 1] = '\0';

			} else {
				PX4_ERR("Failed to get checkpoint name from metadata");
				return false;
			}

			if (successfully_loaded) {
				PX4_INFO("Policy loaded from file %s", path);
				TI input_offset = 0;
				TI input_size = 0;
				rlt::persist::backends::tar::seek(device, reader_group.data, "example/input/data", input_offset, input_size);
				PX4_INFO("Input offset: %d", (int)input_offset);
				TI output_offset = 0;
				TI output_size = 0;
				rlt::persist::backends::tar::seek(device, reader_group.data, "example/output/data", output_offset, output_size);
				PX4_INFO("Output offset: %d", (int)output_offset);

				if (!test_policy(f, input_offset, output_offset)) {
					PX4_ERR("Checkpoint test failed");
					return false;
				}

			} else {
				PX4_ERR("Failed to load policy from file %s", path);
				return false;
			}

			fclose(f);
		}

	} else {
		PX4_INFO("File %s does not exist", path);
		return false;
	}

#else

	strncpy(checkpoint_name, MC_RAPTOR_META_NAMESPACE::name, CHECKPOINT_NAME_LENGTH);

	if (!test_policy()) {
		PX4_ERR("Checkpoint test failed");
		return false;
	}

#endif
	PX4_INFO("Checkpoint name: %s", checkpoint_name);


	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "RAPTOR", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = Raptor::EXT_COMPONENT_REQUEST_ID;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	register_ext_component_request.enable_replace_internal_mode = _param_mc_raptor_offboard.get();
	register_ext_component_request.replace_internal_mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	_register_ext_component_request_pub.publish(register_ext_component_request);

	int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();

	if (imu_gyro_ratemax % POLICY_CONTROL_FREQUENCY_TRAINING != 0) {
		PX4_WARN("IMU_GYRO_RATEMAX=%d Hz is not a multiple of the training frequency (%d Hz)", (int)imu_gyro_ratemax,
			 (int)POLICY_CONTROL_FREQUENCY_TRAINING);
	}

	int32_t force_sync_native = imu_gyro_ratemax / POLICY_CONTROL_FREQUENCY_TRAINING;
	executor.executor.force_sync_native = force_sync_native;
	executor.executor.force_sync_native_initialized = true;
	PX4_INFO("IMU_GYRO_RATEMAX=%d Hz", (int)imu_gyro_ratemax);
	PX4_INFO("POLICY_CONTROL_FREQUENCY_TRAINING=%d Hz", (int)POLICY_CONTROL_FREQUENCY_TRAINING);
	PX4_INFO("Setting force_sync_native = %d Hz / %d Hz = %d", (int)imu_gyro_ratemax, (int)POLICY_CONTROL_FREQUENCY_TRAINING,
		 (int)force_sync_native);

	this->use_internal_reference = _param_mc_raptor_intref.get();

	this->reset();

	return true;
}
template <typename T>
T clip(T x, T max, T min)
{
	if (x > max) {
		return max;
	}

	if (x < min) {
		return min;
	}

	return x;
}
template <typename T>
void quaternion_multiplication(T q1[4], T q2[4], T q_res[4])
{
	q_res[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q_res[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q_res[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	q_res[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}
template <typename T>
void quaternion_conjugate(T q[4], T q_res[4])
{
	q_res[0] = +q[0];
	q_res[1] = -q[1];
	q_res[2] = -q[2];
	q_res[3] = -q[3];
}
template <typename T>
void quaternion_to_rotation_matrix(T q[4], T R[9])
{
	// row-major
	T qw = q[0];
	T qx = q[1];
	T qy = q[2];
	T qz = q[3];

	R[0] = 1 - 2 * qy * qy - 2 * qz * qz;
	R[1] =     2 * qx * qy - 2 * qw * qz;
	R[2] =     2 * qx * qz + 2 * qw * qy;
	R[3] =     2 * qx * qy + 2 * qw * qz;
	R[4] = 1 - 2 * qx * qx - 2 * qz * qz;
	R[5] =     2 * qy * qz - 2 * qw * qx;
	R[6] =     2 * qx * qz - 2 * qw * qy;
	R[7] =     2 * qy * qz + 2 * qw * qx;
	R[8] = 1 - 2 * qx * qx - 2 * qy * qy;
}

template <typename T>
void rotate_vector(T R[9], T v[3], T v_rotated[3])
{
	v_rotated[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
	v_rotated[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
	v_rotated[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

void Raptor::observe(rl_tools::inference::applications::l2f::Observation<EXECUTOR_SPEC> &observation)
{
	// converting from FRD to FLU
	T Rt_inv[9];

	{
		// Orientation
		// FRD to FLU
		T q_target[4];
		q_target[0] = cosf(0.5f * _trajectory_setpoint.yaw); // minus because the setpoint yaw is in NED
		q_target[1] = 0;
		q_target[2] = 0;
		q_target[3] = sinf(0.5f * _trajectory_setpoint.yaw);

		T qt[4], qtc[4], qr[4];
		qt[0] = +q_target[0]; // conjugate to build the difference between setpoint and current
		qt[1] = +q_target[1];
		qt[2] = -q_target[2];
		qt[3] = -q_target[3];
		quaternion_conjugate(qt, qtc);
		quaternion_to_rotation_matrix(qtc, Rt_inv);

		qr[0] = +_vehicle_attitude.q[0];
		qr[1] = +_vehicle_attitude.q[1];
		qr[2] = -_vehicle_attitude.q[2];
		qr[3] = -_vehicle_attitude.q[3];
		// qr = qt * qd
		// qd = qt' * qr
		T qd[4];
		quaternion_multiplication(qtc, qr, qd);

		observation.orientation[0] = qd[0];
		observation.orientation[1] = qd[1];
		observation.orientation[2] = qd[2];
		observation.orientation[3] = qd[3];
	}

	{
		// Position
		T p[3], pt[3]; // FLU
		p[0] = +(position[0] - _trajectory_setpoint.position[0]);
		p[1] = -(position[1] - _trajectory_setpoint.position[1]);
		p[2] = -(position[2] - _trajectory_setpoint.position[2]);
		rotate_vector(Rt_inv, p, pt); // The position and velocity error are in the target frame
		observation.position[0] = clip(pt[0], max_position_error, -max_position_error);
		observation.position[1] = clip(pt[1], max_position_error, -max_position_error);
		observation.position[2] = clip(pt[2], max_position_error, -max_position_error);
	}
	{
		// Linear Velocity
		T v[3], vt[3];
		v[0] = +(linear_velocity[0] - _trajectory_setpoint.velocity[0]);
		v[1] = -(linear_velocity[1] - _trajectory_setpoint.velocity[1]);
		v[2] = -(linear_velocity[2] - _trajectory_setpoint.velocity[2]);
		rotate_vector(Rt_inv, v, vt);
		observation.linear_velocity[0] = clip(vt[0], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[1] = clip(vt[1], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[2] = clip(vt[2], max_velocity_error, -max_velocity_error);
	}
	{
		// Angular Velocity
		observation.angular_velocity[0] = +_vehicle_angular_velocity.xyz[0];
		observation.angular_velocity[1] = -_vehicle_angular_velocity.xyz[1];
		observation.angular_velocity[2] = -_vehicle_angular_velocity.xyz[2];
	}

	for (TI action_i = 0; action_i < EXECUTOR_CONFIG::OUTPUT_DIM; action_i++) {
		observation.previous_action[action_i] = this->previous_action[action_i];
	}
}


void Raptor::updateArmingCheckReply()
{
	if (flightmode_state == FlightModeState::CONFIGURED) {
		if (_arming_check_request_sub.updated()) {
			arming_check_request_s arming_check_request;
			_arming_check_request_sub.copy(&arming_check_request);
			arming_check_reply_s arming_check_reply;
			arming_check_reply.timestamp = hrt_absolute_time();
			arming_check_reply.request_id = arming_check_request.request_id;
			arming_check_reply.registration_id = ext_component_arming_check_id;
			arming_check_reply.health_component_index = arming_check_reply.HEALTH_COMPONENT_INDEX_NONE;
			arming_check_reply.num_events = 0;
			arming_check_reply.can_arm_and_run = can_arm;
			arming_check_reply.mode_req_angular_velocity = true;
			arming_check_reply.mode_req_local_position = true;
			arming_check_reply.mode_req_attitude = true;
			arming_check_reply.mode_req_local_alt = true;
			arming_check_reply.mode_req_home_position = false;
			arming_check_reply.mode_req_mission = false;
			arming_check_reply.mode_req_global_position = false;
			arming_check_reply.mode_req_prevent_arming = false;
			arming_check_reply.mode_req_manual_control = false;
			_arming_check_reply_pub.publish(arming_check_reply);
		}
	}
}


void Raptor::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();

		if (flightmode_state >= FlightModeState::REGISTERED) {
			unregister_ext_component_s unregister_ext_component{};
			unregister_ext_component.timestamp = hrt_absolute_time();
			strncpy(unregister_ext_component.name, "RAPTOR", sizeof(unregister_ext_component.name) - 1);
			unregister_ext_component.arming_check_id = ext_component_arming_check_id;
			unregister_ext_component.mode_id = ext_component_mode_id;
			unregister_ext_component.mode_executor_id = -1;
			_unregister_ext_component_pub.publish(unregister_ext_component);
		}

		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	register_ext_component_reply_s register_ext_component_reply;

	if (_register_ext_component_reply_sub.update(&register_ext_component_reply)) {
		if (register_ext_component_reply.request_id == Raptor::EXT_COMPONENT_REQUEST_ID && register_ext_component_reply.success) {
			ext_component_arming_check_id = register_ext_component_reply.arming_check_id;
			ext_component_mode_id = register_ext_component_reply.mode_id;
			flightmode_state = FlightModeState::REGISTERED;
			PX4_INFO("Raptor mode registration successful, arming_check_id: %d, mode_id: %d", ext_component_arming_check_id, ext_component_mode_id);
		}
	}

	if (flightmode_state == FlightModeState::REGISTERED) {
		vehicle_control_mode_s config_control_setpoints{};
		config_control_setpoints.timestamp = hrt_absolute_time();
		config_control_setpoints.source_id = ext_component_mode_id;
		config_control_setpoints.flag_multicopter_position_control_enabled = false;
		config_control_setpoints.flag_control_manual_enabled = false;
		config_control_setpoints.flag_control_offboard_enabled = false;
		config_control_setpoints.flag_control_position_enabled = false;
		config_control_setpoints.flag_control_climb_rate_enabled = false;
		config_control_setpoints.flag_control_allocation_enabled = false;
		config_control_setpoints.flag_control_termination_enabled = true;
		_config_control_setpoints_pub.publish(config_control_setpoints);
		flightmode_state = FlightModeState::CONFIGURED;
		PX4_INFO("Raptor mode configuration sent");
	}


	perf_count(_loop_interval_perf);

	perf_begin(_loop_perf);
	hrt_abstime current_time = hrt_absolute_time();

	raptor_status_s status{};
	status.timestamp = current_time;
	status.timestamp_sample = current_time;
	status.exit_reason = raptor_status_s::EXIT_REASON_NONE;
	status.substep = 0;
	status.active = false;
	status.control_interval = NAN;
	status.trajectory_setpoint_dt_mean = NAN;
	status.trajectory_setpoint_dt_max = NAN;
	status.trajectory_setpoint_dt_max_since_activation = NAN;

	if (trajectory_setpoint_dts_full || trajectory_setpoint_dt_index > 0) {
		float trajectory_setpoint_dt_mean = 0;
		float trajectory_setpoint_dt_max = 0;

		for (TI i = 0; i < (trajectory_setpoint_dts_full ? NUM_TRAJECTORY_SETPOINT_DTS : trajectory_setpoint_dt_index); i++) {
			TI index = trajectory_setpoint_dts_full ? i : trajectory_setpoint_dt_index - 1 - i;
			trajectory_setpoint_dt_mean += trajectory_setpoint_dts[index];

			if (trajectory_setpoint_dts[index] > trajectory_setpoint_dt_max) {
				trajectory_setpoint_dt_max = trajectory_setpoint_dts[index];
			}
		}

		if (trajectory_setpoint_dt_max > trajectory_setpoint_dt_max_since_reset) {
			trajectory_setpoint_dt_max_since_reset = trajectory_setpoint_dt_max;
		}

		trajectory_setpoint_dt_mean /= NUM_TRAJECTORY_SETPOINT_DTS;
		status.trajectory_setpoint_dt_mean = trajectory_setpoint_dt_mean;
		status.trajectory_setpoint_dt_max = trajectory_setpoint_dt_max;
		status.trajectory_setpoint_dt_max_since_activation = trajectory_setpoint_dt_max_since_reset;
	}

	status.subscription_update_vehicle_status = _vehicle_status_sub.update(&_vehicle_status);

	if (status.subscription_update_vehicle_status) {
		timestamp_last_vehicle_status = current_time;
		timestamp_last_vehicle_status_set = true;
	}

	bool next_active = timestamp_last_vehicle_status_set && _vehicle_status.nav_state == ext_component_mode_id;

	if (!previous_active && next_active) {
		this->reset();
		PX4_INFO("Resetting Inference Executor (Recurrent State)");

	} else {
		if (previous_active && !next_active) {
			PX4_INFO("inactive");
		}
	}

	bool angular_velocity_update = false;
	status.subscription_update_angular_velocity = _vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);

	if (status.subscription_update_angular_velocity) {
		timestamp_last_angular_velocity = current_time;
		timestamp_last_angular_velocity_set = true;
		angular_velocity_update = true;
	}

	status.timestamp_last_vehicle_angular_velocity = current_time;
	status.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	status.subscription_update_local_position = _vehicle_local_position_sub.update(&_vehicle_local_position);

	if (status.subscription_update_local_position) {
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
	}

	status.timestamp_last_vehicle_local_position = current_time;

	status.subscription_update_attitude = _vehicle_attitude_sub.update(&_vehicle_attitude);

	if (status.subscription_update_attitude) {
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
	}

	status.timestamp_last_vehicle_attitude = timestamp_last_attitude;

	trajectory_setpoint_s temp_trajectory_setpoint;
	bool use_external_reference = !use_internal_reference;
	status.subscription_update_trajectory_setpoint = use_external_reference && _trajectory_setpoint_sub.update(&temp_trajectory_setpoint);

	if (status.subscription_update_trajectory_setpoint) {
		if (
			PX4_ISFINITE(temp_trajectory_setpoint.position[0]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.position[1]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.position[2]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.yaw) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[0]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[1]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[2]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.yawspeed)
		) {
			if (timestamp_last_trajectory_setpoint_set) {
				trajectory_setpoint_dts[trajectory_setpoint_dt_index] = current_time - timestamp_last_trajectory_setpoint;
				trajectory_setpoint_dt_index++;

				if (trajectory_setpoint_dt_index == NUM_TRAJECTORY_SETPOINT_DTS) {
					if (next_active && !trajectory_setpoint_dts_full) {
						PX4_INFO("trajectory_setpoint_dts_full");
					}

					trajectory_setpoint_dts_full = true;
					trajectory_setpoint_dt_index = 0;
				}
			}

			timestamp_last_trajectory_setpoint_set = true;
			status.timestamp_last_trajectory_setpoint = current_time;
			timestamp_last_trajectory_setpoint = current_time;
			_trajectory_setpoint = temp_trajectory_setpoint;

		} else {
			trajectory_setpoint_invalid_count++;

			if (next_active && trajectory_setpoint_invalid_count % TRAJECTORY_SETPOINT_INVALID_COUNT_WARNING_INTERVAL == 0) {
				PX4_WARN("trajectory_setpoint invalid, count: %d", (int)trajectory_setpoint_invalid_count);
			}
		}
	}

	constexpr bool PUBLISH_NON_COMPLETE_STATUS = true;

	if (!angular_velocity_update) {
		status.exit_reason = raptor_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		updateArmingCheckReply();
		return;
	}

	if (!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set) {
		status.exit_reason = raptor_status_s::EXIT_REASON_NOT_ALL_OBSERVATIONS_SET;
		status.vehicle_angular_velocity_stale = !timestamp_last_angular_velocity_set;
		status.vehicle_local_position_stale = !timestamp_last_local_position_set;
		status.vehicle_attitude_stale = !timestamp_last_attitude_set;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		can_arm = false;
		updateArmingCheckReply();
		return;
	}

	if ((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY) {
		status.exit_reason = raptor_status_s::EXIT_REASON_ANGULAR_VELOCITY_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("angular velocity timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return;
	}

	if ((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_LOCAL_POSITION) {
		status.exit_reason = raptor_status_s::EXIT_REASON_LOCAL_POSITION_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("local position timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return;

	} else {
		position[0] = _vehicle_local_position.x;
		position[1] = _vehicle_local_position.y;
		position[2] = _vehicle_local_position.z;
		linear_velocity[0] = _vehicle_local_position.vx;
		linear_velocity[1] = _vehicle_local_position.vy;
		linear_velocity[2] = _vehicle_local_position.vz;
	}

	// position and linear_velocity are guaranteed to be set after this point
	auto previous_internal_reference = internal_reference;
	internal_reference = static_cast<InternalReference>(_param_mc_raptor_intref.get());
	bool internal_reference_changed = previous_internal_reference != internal_reference;

	if (internal_reference_changed) {
		PX4_INFO("internal reference changed from %d to %d", (int)previous_internal_reference, (int)internal_reference);
	}

	if (use_internal_reference && internal_reference != InternalReference::NONE) {
		if (next_active && (!previous_active || internal_reference_changed || internal_reference_params_changed)) {
			internal_reference_activation_position[0] = position[0];
			internal_reference_activation_position[1] = - position[1];
			internal_reference_activation_position[2] = - position[2];
			internal_reference_activation_orientation[0] = _vehicle_attitude.q[0];
			internal_reference_activation_orientation[1] = _vehicle_attitude.q[1];
			internal_reference_activation_orientation[2] = -_vehicle_attitude.q[2];
			internal_reference_activation_orientation[3] = -_vehicle_attitude.q[3];
			internal_reference_activation_time = current_time;
			PX4_INFO("internal reference activated at: %f %f %f", (double)internal_reference_activation_position[0],
				 (double)internal_reference_activation_position[1], (double)internal_reference_activation_position[2]);
			internal_reference_params_changed = false;
		}

		Setpoint setpoint{};

		if (internal_reference == InternalReference::LISSAJOUS) {
			setpoint = lissajous(static_cast<T>(current_time - internal_reference_activation_time) / 1000000, lissajous_params);

		} else {
			PX4_ERR("internal reference type not supported");
		}

		auto &q = internal_reference_activation_orientation;
		matrix::Quatf q_activation_frame(q[0], q[1], q[2], q[3]);
		matrix::Vector3f position_activation_frame = q_activation_frame.rotateVector(matrix::Vector3f(setpoint.position[0], setpoint.position[1],
				setpoint.position[2]));
		matrix::Vector3f linear_velocity_activation_frame = q_activation_frame.rotateVector(matrix::Vector3f(setpoint.linear_velocity[0],
				setpoint.linear_velocity[1], setpoint.linear_velocity[2]));

		_trajectory_setpoint.position[0] = +(internal_reference_activation_position[0] + position_activation_frame(0));
		_trajectory_setpoint.position[1] = -(internal_reference_activation_position[1] + position_activation_frame(1));
		_trajectory_setpoint.position[2] = -(internal_reference_activation_position[2] + position_activation_frame(2));
		_trajectory_setpoint.yaw = - atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) - setpoint.yaw;
		_trajectory_setpoint.velocity[0] = +linear_velocity_activation_frame(0);
		_trajectory_setpoint.velocity[1] = -linear_velocity_activation_frame(1);
		_trajectory_setpoint.velocity[2] = -linear_velocity_activation_frame(2);
		_trajectory_setpoint.yawspeed = -setpoint.yaw_rate;
		timestamp_last_trajectory_setpoint_set = true;
		status.timestamp_last_trajectory_setpoint = current_time;
		timestamp_last_trajectory_setpoint = current_time;

		status.internal_reference_position[0] = _trajectory_setpoint.position[0];
		status.internal_reference_position[1] = _trajectory_setpoint.position[1];
		status.internal_reference_position[2] = _trajectory_setpoint.position[2];
		status.internal_reference_linear_velocity[0] = _trajectory_setpoint.velocity[0];
		status.internal_reference_linear_velocity[1] = _trajectory_setpoint.velocity[1];
		status.internal_reference_linear_velocity[2] = _trajectory_setpoint.velocity[2];
	}

	if ((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE) {
		status.exit_reason = raptor_status_s::EXIT_REASON_ATTITUDE_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("attitude timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return;
	}

	timeout_message_sent = false;

	// is ready to control at this point
	can_arm = true;
	updateArmingCheckReply();

	if (!timestamp_last_trajectory_setpoint_set || use_internal_reference
	    || (current_time - timestamp_last_trajectory_setpoint) > TRAJECTORY_SETPOINT_TIMEOUT) {
		status.trajectory_setpoint_stale = true;

		if (!previous_trajectory_setpoint_stale || (!previous_active && next_active)) {
			_trajectory_setpoint.position[0] = position[0];
			_trajectory_setpoint.position[1] = position[1];
			_trajectory_setpoint.position[2] = position[2];
			auto &q = _vehicle_attitude.q;
			_trajectory_setpoint.yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
			_trajectory_setpoint.velocity[0] = 0;
			_trajectory_setpoint.velocity[1] = 0;
			_trajectory_setpoint.velocity[2] = 0;
			_trajectory_setpoint.yawspeed = 0;

			if (!previous_trajectory_setpoint_stale) {
				PX4_WARN("trajectory_setpoint turned stale at: %f %f %f, yaw: %f %llu / %llu us", (double)position[0], (double)position[1],
					 (double)position[2],
					 (double)_trajectory_setpoint.yaw, (unsigned long long)(current_time - timestamp_last_trajectory_setpoint),
					 (unsigned long long)(TRAJECTORY_SETPOINT_TIMEOUT));

			} else {
				PX4_WARN("trajectory_setpoint reset due to activation at: %f %f %f, yaw: %f", (double)position[0], (double)position[1], (double)position[2],
					 (double)_trajectory_setpoint.yaw);
			}
		}

		previous_trajectory_setpoint_stale = true;

	} else {
		if (previous_trajectory_setpoint_stale) {
			PX4_WARN("trajectory_setpoint turned non-stale at: %f %f %f", (double)position[0], (double)position[1], (double)position[2]);
			previous_trajectory_setpoint_stale = false;
		}

		status.trajectory_setpoint_stale = false;
	}





	rl_tools::inference::applications::l2f::Observation<EXECUTOR_SPEC> observation;
	rl_tools::inference::applications::l2f::Action<EXECUTOR_SPEC> action;
	observe(observation);
	hrt_abstime nanoseconds = current_time * 1000;
	auto executor_status = rl_tools::control(device, executor, nanoseconds, policy, observation, action, rng);

	if (!executor_status.OK) {
		if (executor_status.TIMESTAMP_INVALID) {
			PX4_ERR("RLtools executor error: Timestamp invalid");
		}

		if (executor_status.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP) {
			PX4_ERR("RLtools executor error: Last control timestamp %llu greater than last observation timestamp %llu",
				(unsigned long long)executor.executor.last_control_timestamp, (unsigned long long)executor.executor.last_observation_timestamp);
		}
	}

	if (executor_status.source != decltype(executor_status.source)::CONTROL) {
		// status.exit_reason = raptor_status_s::EXIT_REASON_EXECUTOR_STATUS_SOURCE_NOT_CONTROL;
		// if constexpr(PUBLISH_NON_COMPLETE_STATUS){
		// 	_raptor_status_pub.publish(status);
		// }
		// Exit early if it is not time to control
		return;
	}


	status.active = next_active;


	// no return after this point!

	raptor_input_s input_msg;
	input_msg.active = status.active;
	static_assert(raptor_input_s::ACTION_DIM == EXECUTOR_CONFIG::OUTPUT_DIM);
	input_msg.timestamp = current_time;
	input_msg.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	for (TI dim_i = 0; dim_i < 3; dim_i++) {
		input_msg.position[dim_i] = observation.position[dim_i];
		input_msg.orientation[dim_i] = observation.orientation[dim_i];
		input_msg.linear_velocity[dim_i] = observation.linear_velocity[dim_i];
		input_msg.angular_velocity[dim_i] = observation.angular_velocity[dim_i];
	}

	input_msg.orientation[3] = observation.orientation[3];

	for (TI dim_i = 0; dim_i < EXECUTOR_CONFIG::OUTPUT_DIM; dim_i++) {
		input_msg.previous_action[dim_i] = observation.previous_action[dim_i];
	}

	_raptor_input_pub.publish(input_msg);
	_raptor_status_pub.publish(status);

	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	for (TI action_i = 0; action_i < actuator_motors_s::NUM_CONTROLS; action_i++) {
		if (action_i < EXECUTOR_CONFIG::OUTPUT_DIM) {
			T value = action.action[action_i];
			this->previous_action[action_i] = value;
			value = (value + 1) / 2;
			constexpr T training_min = 0;
			constexpr T training_max = 1.0;
			T scaled_value = (training_max - training_min) * value + training_min;
			actuator_motors.control[action_i] = scaled_value;

		} else {
			actuator_motors.control[action_i] = NAN;
		}
	}

	if constexpr(Raptor::REMAP_FROM_CRAZYFLIE) {
		actuator_motors_s temp = actuator_motors;
		temp.control[0] = actuator_motors.control[0];
		temp.control[1] = actuator_motors.control[2];
		temp.control[2] = actuator_motors.control[3];
		temp.control[3] = actuator_motors.control[1];
		actuator_motors = temp;
	}

	if (status.active) {
		_actuator_motors_pub.publish(actuator_motors);
	}

	perf_end(_loop_perf);
	previous_active = next_active;

	if (executor_status.source == decltype(executor_status.source)::CONTROL) {
		if (executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE) {
			this->last_intermediate_status = executor_status;
			this->last_intermediate_status_set = true;

		} else if (executor_status.step_type == decltype(executor_status.step_type)::NATIVE) {
			this->last_native_status = executor_status;
			this->last_native_status_set = true;
		}
	}

	if (!this->timestamp_last_policy_frequency_check_set
	    || (current_time - timestamp_last_policy_frequency_check) > POLICY_FREQUENCY_CHECK_INTERVAL) {
		if (this->timestamp_last_policy_frequency_check_set) {
			if (last_intermediate_status_set) {
				if (!this->last_intermediate_status.timing_bias.OK || !this->last_intermediate_status.timing_jitter.OK) {
					PX4_WARN("Raptor: INTERMEDIATE: BIAS %fx JITTER %fx", (double)this->last_intermediate_status.timing_bias.MAGNITUDE,
						 (double)this->last_intermediate_status.timing_jitter.MAGNITUDE);

				} else {
					if (ENABLE_CONTROL_FREQUENCY_INFO && this->policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0) {
						PX4_INFO("Raptor: INTERMEDIATE: BIAS %fx JITTER %fx", (double)this->last_intermediate_status.timing_bias.MAGNITUDE,
							 (double)this->last_intermediate_status.timing_jitter.MAGNITUDE);
					}
				}
			}

			if (last_native_status_set) {
				if (!this->last_native_status.timing_bias.OK || !this->last_native_status.timing_jitter.OK) {
					PX4_WARN("Raptor: NATIVE: BIAS %fx JITTER %fx", (double)this->last_native_status.timing_bias.MAGNITUDE,
						 (double)this->last_native_status.timing_jitter.MAGNITUDE);

				} else {
					if (ENABLE_CONTROL_FREQUENCY_INFO && this->policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0) {
						PX4_INFO("Raptor: NATIVE: BIAS %fx JITTER %fx", (double)this->last_native_status.timing_bias.MAGNITUDE,
							 (double)this->last_native_status.timing_jitter.MAGNITUDE);
					}
				}
			}
		}

		this->num_healthy_executor_statii_intermediate = 0;
		this->num_non_healthy_executor_statii_intermediate = 0;
		this->num_healthy_executor_statii_native = 0;
		this->num_non_healthy_executor_statii_native = 0;
		this->num_statii = 0;
		this->timestamp_last_policy_frequency_check = current_time;
		this->timestamp_last_policy_frequency_check_set = true;
		this->policy_frequency_check_counter++;
	}

	this->num_statii++;
	this->num_healthy_executor_statii_intermediate += executor_status.OK && executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE;
	this->num_non_healthy_executor_statii_intermediate += (!executor_status.OK)
			&& executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE;
	this->num_healthy_executor_statii_native += executor_status.OK && executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::NATIVE;
	this->num_non_healthy_executor_statii_native += (!executor_status.OK)
			&& executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::NATIVE;
}

int Raptor::task_spawn(int argc, char *argv[])
{
	Raptor *instance = new Raptor();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int Raptor::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_loop_interval_policy_perf);
	PX4_INFO_RAW("Checkpoint: %s\n", checkpoint_name);
	return 0;
}

int Raptor::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[0], "intref") == 0) {
		if (strcmp(argv[1], "lissajous") == 0) {
			// Usage: mc_raptor intref lissajous <A> <B> <C> <fa> <fb> <fc> <duration> <ramp>
			if (argc != 10) {
				PX4_ERR("Usage: mc_raptor intref lissajous <A> <B> <C> <fa> <fb> <fc> <duration> <ramp>");
				return PX4_ERROR;
			}

			Raptor *instance = get_instance<Raptor>(desc);

			if (instance == nullptr) {
				PX4_ERR("mc_raptor is not running");
				return PX4_ERROR;
			}

			instance->lissajous_params.A = strtof(argv[2], nullptr);
			instance->lissajous_params.B = strtof(argv[3], nullptr);
			instance->lissajous_params.C = strtof(argv[4], nullptr);
			instance->lissajous_params.a = strtof(argv[5], nullptr);
			instance->lissajous_params.b = strtof(argv[6], nullptr);
			instance->lissajous_params.c = strtof(argv[7], nullptr);
			instance->lissajous_params.duration = strtof(argv[8], nullptr);
			instance->lissajous_params.ramp_duration = strtof(argv[9], nullptr);
			instance->internal_reference_params_changed = true;

			PX4_INFO("Lissajous params set: A=%.2f B=%.2f C=%.2f fa=%.2f fb=%.2f fc=%.2f duration=%.2f ramp=%.2f \n",
				 (double)instance->lissajous_params.A,
				 (double)instance->lissajous_params.B,
				 (double)instance->lissajous_params.C,
				 (double)instance->lissajous_params.a,
				 (double)instance->lissajous_params.b,
				 (double)instance->lissajous_params.c,
				 (double)instance->lissajous_params.duration,
				 (double)instance->lissajous_params.ramp_duration);


			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int Raptor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RAPTOR Policy Flight Mode

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_raptor", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("intref", "Modify internal reference");
	PRINT_MODULE_USAGE_ARG("lissajous", "Set Lissajous trajectory parameters", false);
	PRINT_MODULE_USAGE_ARG("<A>", "Amplitude X [m]", false);
	PRINT_MODULE_USAGE_ARG("<B>", "Amplitude Y [m]", false);
	PRINT_MODULE_USAGE_ARG("<C>", "Amplitude Z [m]", false);
	PRINT_MODULE_USAGE_ARG("<fa>", "Frequency a", false);
	PRINT_MODULE_USAGE_ARG("<fb>", "Frequency b", false);
	PRINT_MODULE_USAGE_ARG("<fc>", "Frequency c", false);
	PRINT_MODULE_USAGE_ARG("<duration>", "Total duration [s]", false);
	PRINT_MODULE_USAGE_ARG("<ramp>", "Ramp duration [s]", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_raptor_main(int argc, char *argv[])
{
	return ModuleBase::main(Raptor::desc, argc, argv);
}
