#include "ControlAllocator.hpp"
#include "ControlAllocation.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <cmath>
#include <matrix/math.hpp>
#include <unistd.h>
#include <matrix/math.hpp>

using namespace matrix;
using namespace time_literals;


ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			// rover_differential_control does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

// Funções para calcular a pseudo-inversa de uma matriz
matrix::Matrix<float, 6, 2> pseudoInverse(const matrix::Matrix<float, 2, 6>& mat) {
    matrix::Matrix<float, 6, 2> mat_T = mat.transpose();
    matrix::SquareMatrix<float, 2> mat_T_mat = mat * mat_T;
    matrix::SquareMatrix<float, 2> mat_T_mat_inv = mat_T_mat.I(); // Inversa de mat_T * mat

    return mat_T * mat_T_mat_inv;
}

matrix::Matrix<float, 6, 5> pseudoInverse2(const matrix::Matrix<float, 5, 6>& mat) {
    matrix::Matrix<float, 6, 5> mat_T = mat.transpose();
    matrix::SquareMatrix<float, 5> mat_T_mat = mat * mat_T;
    matrix::SquareMatrix<float, 5> mat_T_mat_inv = mat_T_mat.I(); // Inversa de mat_T * mat

    return mat_T * mat_T_mat_inv;
}

// Set the PWM to motors
void ControlAllocator::setPWM(int motor_index, float actuator_sp) {
	actuator_motors_s actuator_motors;
	_actuator_motors_pub.advertise();

	actuator_motors.control[motor_index] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
	_actuator_motors_pub.publish(actuator_motors);
	actuator_motors.timestamp = hrt_absolute_time();

}

// Set the angle of a servo
void ControlAllocator::setServo(int servo_index, float angle) {
	actuator_servos_s actuator_servos;
	_actuator_servos_pub.advertise();

	actuator_servos.control[servo_index] = PX4_ISFINITE(angle) ? angle : NAN;
	_actuator_servos_pub.publish(actuator_servos);
}

// Função para implementar a FCA
void ControlAllocator::HexacopterTiltedAllocation(const matrix::Vector<float, 6>& torques, matrix::Vector<float, 16> actuator_sp) {
	// Inicializar variáveis necessárias
	float IN_ANGLE_1 = math::degrees((actuator_sp)(6));
	float IN_ANGLE_2 = math::degrees((actuator_sp)(7));

	float ag1_c = std::cos(IN_ANGLE_1);
	float ag2_c = std::cos(IN_ANGLE_2);

	float f1 = 0.5f;
	float f2 = std::sqrt(3.0f) / 2.0f;

	float k1 = 7.81f;
	float k2 = 0.0001f*k1;
	float l = 0.25f;

	matrix::Vector<float, 2> SEN;
	matrix::Vector<float, 6> OUT2;

	// Verificar se todos os elementos de actuator_sp são zero
	bool all_actuators_zero = true;
	const float epsilon = 1e-6; // Tolerância para comparação de ponto flutuante
	for (int i = 0; i < 6; ++i) {
		if (std::fabs((actuator_sp)(i)) > epsilon) {
		all_actuators_zero = false;
		break;
		}
	}

	if (!all_actuators_zero) {
		matrix::Matrix<float, 2, 6> M_TiltQuad2;
		M_TiltQuad2(0, 0) = k1 * (actuator_sp)(0);
		M_TiltQuad2(0, 1) = k1 * (actuator_sp)(1);
		M_TiltQuad2(0, 2) = 0;
		M_TiltQuad2(0, 3) = 0;
		M_TiltQuad2(0, 4) = 0;
		M_TiltQuad2(0, 5) = 0;

		M_TiltQuad2(1, 0) = -k1 * (actuator_sp)(0) * l;
		M_TiltQuad2(1, 1) = k1 * (actuator_sp)(1) * l;
		M_TiltQuad2(1, 2) = -k2 * (actuator_sp)(2) + k2 * (actuator_sp)(3) + k2 * (actuator_sp)(4) - k2 * (actuator_sp)(5) - k2 * (actuator_sp)(0) * ag1_c + k2 * (actuator_sp)(1) * ag2_c;
		M_TiltQuad2(1, 3) = 0;
		M_TiltQuad2(1, 4) = 0;
		M_TiltQuad2(1, 5) = 0;

		matrix::Matrix<float, 6, 2> M_Inv_TQuad2 = pseudoInverse(M_TiltQuad2);
		matrix::Vector<float, 2> torques_subset;

		torques_subset(0) = torques(0);
		torques_subset(1) = torques(5);

		matrix::Matrix<float, 6, 1> ARCSEN_matrix = M_Inv_TQuad2 * torques_subset;
		matrix::Vector<float, 2> ARCSEN;

		ARCSEN(0) = ARCSEN_matrix(0, 0);
		ARCSEN(1) = ARCSEN_matrix(1, 0);
		SEN(0) = std::asin(ARCSEN(0));
		SEN(1) = std::asin(ARCSEN(1));
	}

	ag1_c = std::cos(SEN(0));
	ag2_c = std::cos(SEN(1));
	float ag1_s = std::sin(SEN(0));
	float ag2_s = std::sin(SEN(1));

	matrix::Matrix<float, 5, 6> M_TiltQuad1;
	M_TiltQuad1(0, 0) = k1 * ag1_s;
	M_TiltQuad1(0, 1) = k1 * ag2_s;
	M_TiltQuad1(0, 2) = 0;
	M_TiltQuad1(0, 3) = 0;
	M_TiltQuad1(0, 4) = 0;
	M_TiltQuad1(0, 5) = 0;

	M_TiltQuad1(1, 0) = k1 * ag1_c;
	M_TiltQuad1(1, 1) = k1 * ag2_c;
	M_TiltQuad1(1, 2) = k1;
	M_TiltQuad1(1, 3) = k1;
	M_TiltQuad1(1, 4) = k1;
	M_TiltQuad1(1, 5) = k1;

	M_TiltQuad1(2, 0) = -k1 * ag1_c * l + k2 * ag1_s;
	M_TiltQuad1(2, 1) = k1 * ag2_c * l - k2 * ag2_s;
	M_TiltQuad1(2, 2) = k1 * l * f1;
	M_TiltQuad1(2, 3) = -k1 * l * f1;
	M_TiltQuad1(2, 4) = -k1 * l * f1;
	M_TiltQuad1(2, 5) = k1 * l * f1;

	M_TiltQuad1(3, 0) = 0;
	M_TiltQuad1(3, 1) = 0;
	M_TiltQuad1(3, 2) = k1 * l * f2;
	M_TiltQuad1(3, 3) = -k1 * l * f2;
	M_TiltQuad1(3, 4) = k1 * l * f2;
	M_TiltQuad1(3, 5) = -k1 * l * f2;

	M_TiltQuad1(4, 0) = -k1 * ag1_s * l - k2 * ag1_c;
	M_TiltQuad1(4, 1) = k1 * ag2_s * l + k2 * ag2_c;
	M_TiltQuad1(4, 2) = -k2;
	M_TiltQuad1(4, 3) = k2;
	M_TiltQuad1(4, 4) = k2;
	M_TiltQuad1(4, 5) = -k2;

	matrix::Matrix<float, 6, 5> M_Inv_TQuad1 = pseudoInverse2(M_TiltQuad1);
	matrix::Vector<float, 5> torques_subset;

	torques_subset(0) = torques(0);
	torques_subset(1) = torques(2);
	torques_subset(2) = torques(3);
	torques_subset(3) = torques(4);
	torques_subset(4) = torques(5);

	OUT2 = M_Inv_TQuad1 * torques_subset;

	// Limitar valores de saida dos atuadores
	for (int i = 0; i < 6; ++i) {
		if (std::isnan(OUT2(i))) {
			OUT2(i) = 0.0f;
		} else if (OUT2(i) > 1.0f) {
			OUT2(i) = 1.0f;
		} else if (OUT2(i) < -1.0f) {
			OUT2(i) = -1.0f;
		}
	}

	// Ajustar os valores e atribuir corretamente aos atuadores
	actuator_sp(0) = 2.0f*OUT2(0);
	actuator_sp(1) = 2.0f*OUT2(1);
	actuator_sp(2) = -2.0f*OUT2(2);
	actuator_sp(3) = -2.0f*OUT2(3);
	actuator_sp(4) = -2.0f*OUT2(4);
	actuator_sp(5) = -2.0f*OUT2(5);
	actuator_sp(6) = -2.0f*SEN(0);
	actuator_sp(7) = 2.0f*SEN(1);

	// Publicar os valores dos atuadores
	setPWM(0, 2.0f*OUT2(0));
	setPWM(1, 2.0f*OUT2(1));
	setPWM(2, -2.0f*OUT2(2));
	setPWM(3, -2.0f*OUT2(3));
	setPWM(4, -2.0f*OUT2(4));
	setPWM(5, -2.0f*OUT2(5));
	setServo(0, -2.0f * SEN(0));
	setServo(1, 2.0f * SEN(1));

	// Visualizar os valores dos atuadores
	//PX4_INFO("Actuator: %f, %f", -(double)actuator_sp(6), -(double)actuator_sp(7));
	//PX4_INFO("Actuator: %f, %f, %f, %f, %f, %f", (double)actuator_sp(0), (double)actuator_sp(1), (double)actuator_sp(2), (double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5));

}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated() && !_armed) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0); // Roll torque
		c[0](1) = _torque_sp(1); // Pitch torque
		c[0](2) = _torque_sp(2); // Yaw torque
		c[0](3) = _thrust_sp(0); // X thrust
		c[0](4) = _thrust_sp(1); // Y thrust
		c[0](5) = _thrust_sp(2); // Z thrust

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0]; // Roll torque
				c[1](1) = vehicle_torque_setpoint.xyz[1]; // Pitch torque
				c[1](2) = vehicle_torque_setpoint.xyz[2]; // Yaw torque
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0]; // X thrust
				c[1](4) = vehicle_thrust_setpoint.xyz[1]; // Y thrust
				c[1](5) = vehicle_thrust_setpoint.xyz[2]; // Z thrust
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Allocate actuator setpoint
			_control_allocation[i]->allocate();

			// Retrieve actuator setpoint
			matrix::Vector<float, 16> actuator_sp = _control_allocation[i]->getActuatorSetpoint();

			// Apply the new allocation matrix
			HexacopterTiltedAllocation(c[i], actuator_sp);

			// Do allocation
			//_control_allocation[i]->allocate();
			//_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			//_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
			//					_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			//if (_has_slew_rate) {
			//	_control_allocation[i]->applySlewRateLimit(dt);
			//}

			//_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);

}

// Publish PWM in motors
void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors() | _handled_motor_failure_bitmask;

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;

		if (stopped_motors & (1u << motors_idx)) {
			actuator_motors.control[motors_idx] = NAN;
		}

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}
	_actuator_motors_pub.publish(actuator_motors);
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {
		if (failure_detector_status.fd_motor) {

			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("  minimum =");
		_control_allocation[i]->getActuatorMin().T().print();
		PX4_INFO("  maximum =");
		_control_allocation[i]->getActuatorMax().T().print();
		PX4_INFO("  Configured actuators: %i", _control_allocation[i]->numConfiguredActuators());
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
