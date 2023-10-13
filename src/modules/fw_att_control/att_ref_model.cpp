#include "att_ref_model.h"

#include <px4_platform_common/log.h>

#include <lib/mathlib/mathlib.h>
#include "lib/slew_rate/SlewRate.hpp"

float REFERENCE_MODEL_DAMPING{1.f};

FixedwingAttitudeReferenceModel::FixedwingAttitudeReferenceModel() :
	ModuleParams(nullptr)
{
	parameters_update();
}

void FixedwingAttitudeReferenceModel::update()
{
	parameters_update();

	/* We check if the attitude setpoint topic got updated externally, put in throughput mode,
	 if attitude reference got updated, but into filter mode. If none, keep the current mode.
	 If both put throughput mode for safety. */
	_att_sp_sub.update();

	if (_att_sp_sub.get().timestamp > _last_att_setpoint_timestamp) {
		if (_mode == ModelMode::MODELMODE_FILTERING) {
			PX4_INFO("FixedWingAttitudeReferenceModel: Switch to throughput mode");
		}

		_mode = ModelMode::MODELMODE_THROUGHPUT;

	} else if (_att_ref_sp_sub.updated()) {
		_att_ref_sp_sub.update();

		if (_mode == ModelMode::MODELMODE_THROUGHPUT) {
			PX4_INFO("FixedWingAttitudeReferenceModel: Switch to filtering mode");
		}

		_mode = ModelMode::MODELMODE_FILTERING;

	}

	hrt_abstime now = hrt_absolute_time();

	_attitiude_rate_feedforward_output = matrix::Vector2f{};
	_attitiude_torque_feedforward_output = matrix::Vector2f{};

	if (_mode == ModelMode::MODELMODE_FILTERING) {

		if (!_is_initialized) {

			_is_initialized = true;
			_roll_ref_model.reset(_att_ref_sp_sub.get().roll_body);
			_pitch_ref_model.reset(_att_ref_sp_sub.get().pitch_body);

		} else {
			if (_att_ref_sp_sub.get().timestamp > _last_update_timestamp) {
				_roll_ref_model.update(static_cast<float>(_att_ref_sp_sub.get().timestamp - _last_update_timestamp) / 1_s,
						       _last_attitude_reference_setpoint.roll_body);
				_pitch_ref_model.update(static_cast<float>(_att_ref_sp_sub.get().timestamp - _last_update_timestamp) / 1_s,
							_last_attitude_reference_setpoint.pitch_body);
				_last_update_timestamp = _att_ref_sp_sub.get().timestamp;
			}

			_roll_ref_model.update(static_cast<float>(math::max(now - _last_update_timestamp, static_cast<hrt_abstime>(0U))) / 1_s,
					       _att_ref_sp_sub.get().roll_body);
			_pitch_ref_model.update(static_cast<float>(math::max(now - _last_update_timestamp, static_cast<hrt_abstime>(0U))) / 1_s,
						_att_ref_sp_sub.get().pitch_body);

		}

		_last_attitude_reference_setpoint = _att_ref_sp_sub.get();

		_last_update_timestamp = now;

		_attitude_setpoint_output = _att_ref_sp_sub.get();
		_attitude_setpoint_output.timestamp = now;

		if (_param_ref_r_en.get()) {
			_attitude_setpoint_output.roll_body = _roll_ref_model.getState();
			_attitiude_rate_feedforward_output(0) = _roll_ref_model.getRate();
			_attitiude_torque_feedforward_output(0) = _roll_ref_model.getAccel();
		}

		if (_param_ref_p_en.get()) {
			_attitude_setpoint_output.pitch_body = _pitch_ref_model.getState();
			_attitiude_rate_feedforward_output(1) = _pitch_ref_model.getRate();
			_attitiude_torque_feedforward_output(1) = _pitch_ref_model.getAccel();

		}

		// Other fields in the attitude setpoints are passed through

		// Publish attitude setpoint for logging
		_att_sp_pub.publish(_attitude_setpoint_output);
		_last_att_setpoint_timestamp = _attitude_setpoint_output.timestamp;

	} else {
		_attitude_setpoint_output = _att_sp_sub.get();
		_roll_ref_model.reset(_att_sp_sub.get().roll_body);
		_pitch_ref_model.reset(_att_sp_sub.get().pitch_body);
		_is_initialized = true;
		_last_update_timestamp = now;
		_last_att_setpoint_timestamp = _att_sp_sub.get().timestamp;
	}
}

void FixedwingAttitudeReferenceModel::reset()
{
	_is_initialized = false;

	if (_mode == ModelMode::MODELMODE_FILTERING) {
		_att_ref_sp_sub.update();
		_attitude_setpoint_output = _att_ref_sp_sub.get();

	} else {
		_att_sp_sub.update();
		_attitude_setpoint_output = _att_sp_sub.get();
	}

	_attitiude_rate_feedforward_output = matrix::Vector2f{};
	_attitiude_torque_feedforward_output = matrix::Vector2f{};
}

void FixedwingAttitudeReferenceModel::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

		_roll_ref_model.setParameters(_param_ref_r_freq.get(), REFERENCE_MODEL_DAMPING, _param_ref_r_vel_limit.get(),
					      _param_ref_r_acc_limit.get());
		_pitch_ref_model.setParameters(_param_ref_p_freq.get(), REFERENCE_MODEL_DAMPING, _param_ref_p_vel_limit.get(),
					       _param_ref_p_acc_limit.get());
	}
}
