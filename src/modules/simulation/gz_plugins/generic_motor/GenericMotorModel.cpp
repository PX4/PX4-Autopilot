/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2022 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "GenericMotorModel.hpp"

#include <mutex>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

// from rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h
/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants.
/// \details
///           Short reveiw of discrete time implementation of first order system
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k) // NOLINT
template <typename T>
class FirstOrderFilter
{
public:
	FirstOrderFilter(double _timeConstantUp, double _timeConstantDown, T _initialState): // NOLINT
		timeConstantUp(_timeConstantUp),
		timeConstantDown(_timeConstantDown),
		previousState(_initialState) {}

	/// \brief    This method will apply a first order filter on the _inputState.
	T UpdateFilter(T _inputState, double _samplingTime)
	{
		T outputState;

		if (_inputState > previousState) {
			// Calcuate the outputState if accelerating.
			double alphaUp = exp(-_samplingTime / timeConstantUp);
			// x(k+1) = Ad*x(k) + Bd*u(k)
			outputState = alphaUp * previousState + (1 - alphaUp) * _inputState;

		} else {
			// Calculate the outputState if decelerating.
			double alphaDown = exp(-_samplingTime / timeConstantDown);
			outputState = alphaDown * previousState + (1 - alphaDown) * _inputState;
		}

		previousState = outputState;
		return outputState;
	}

	~FirstOrderFilter() = default;

protected:
	double timeConstantUp;
	double timeConstantDown;
	T previousState;
};

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Constants for specifying clockwise (kCw) and counter-clockwise (kCcw)
/// directions of rotation.
namespace turning_direction
{
static const int kCcw = 1;
static const int kCw = -1;
}  // namespace turning_direction

/// \brief Type of input command to motor.
enum class MotorType {
	kVelocity,
	kPosition,
	kForce,
	kForcePolynomial
};

/// \brief Type of control methology
enum class ControlMethod {
	kRPM,
	kDutyCycle
};

class gz::sim::systems::GenericMotorModelPrivate
{
	/// \brief Callback for actuator commands.
public: void OnActuatorMsg(const msgs::Actuators &_msg);

	/// \brief Apply link forces and moments based on propeller state.
public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

	/// \brief Joint Entity
public: Entity jointEntity;

	/// \brief Joint name
public: std::string jointName;

	/// \brief Link Entity
public: Entity linkEntity;

	/// \brief Link name
public: std::string linkName;

	/// \brief Parent link Entity
public: Entity parentLinkEntity;

	/// \brief Parent link name
public: std::string parentLinkName;

	/// \brief Model interface
public: Model model{kNullEntity};

	/// \brief Topic for actuator commands.
public: std::string commandSubTopic;

	/// \brief Topic namespace.
public: std::string robotNamespace;

	/// \brief Sampling time (from motor_model.hpp).
public: double samplingTime = 0.01;

	/// \brief Index of motor in Actuators msg on multirotor_base.
public: int actuatorNumber = 0;

	/// \brief Turning direction of the motor.
public: int turningDirection = turning_direction::kCw;

	/// \brief Type of input command to motor.
public: MotorType motorType = MotorType::kVelocity;

	/// \brief Type of input command to motor.
public: ControlMethod controlMethod = ControlMethod::kRPM;

	/// \brief Maximum rotational velocity command with units of rad/s.
	/// The default value is taken from gazebo_motor_model.h
	/// and is approximately 8000 revolutions / minute (rpm).
public: double maxRotVelocity = 838.0;

	/// \brief Minimum rotor velocity, below which the rotor is considered
	/// to be stopped. Simulates minimum velocity achieved by the motor
public: double minCommand = 0.0;

	/// \brief Maximum duty cycle value
public: double maxDutyCycle = 0.0;

	/// \brief Minimum duty cycle value
public: double minDutyCycle = 0.0;

	/// \brief Bidirectional-capable motor. Set to true if it mimics an ESC
	/// with bidirectional control. Zero input will be middle of max and min
	/// duty cycles.
public: int bidirectionalMotor = 0;

	/// \brief Moment constant for computing drag torque based on thrust
	/// with units of length (m).
	/// The default value is taken from gazebo_motor_model.h
public: double momentConstant = 0.016;

	/// \brief Thrust coefficient for propeller with units of N / (rad/s)^2.
	/// The default value is taken from gazebo_motor_model.h
public: double motorConstant = 8.54858e-06;

	/// \brief Reference input to motor. For MotorType kVelocity, this
	/// is the reference angular velocity in rad/s.
public: double refMotorInput = 0.0;

	/// \brief Rolling moment coefficient with units of N*m / (m/s^2).
	/// The default value is taken from gazebo_motor_model.h
public: double rollingMomentCoefficient = 1.0e-6;

	/// \brief Rotor drag coefficient for propeller with units of N / (m/s^2).
	/// The default value is taken from gazebo_motor_model.h
public: double rotorDragCoefficient = 1.0e-4;

	/// \brief Large joint velocities can cause problems with aliasing,
	/// so the joint velocity used by the physics engine is reduced
	/// this factor, while the larger value is used for computing
	/// propeller thrust.
	/// The default value is taken from gazebo_motor_model.h
public: double rotorVelocitySlowdownSim = 10.0;

	/// \brief Time constant for rotor deceleration.
	/// The default value is taken from gazebo_motor_model.h
public: double timeConstantDown = 1.0 / 40.0;

	/// \brief Time constant for rotor acceleration.
	/// The default value is taken from gazebo_motor_model.h
public: double timeConstantUp = 1.0 / 80.0;

	/// \brief Filter on rotor velocity that has different time constants
	/// for increasing and decreasing values.
public: std::unique_ptr<FirstOrderFilter<double>> rotorVelocityFilter;

	/// \brief Received Actuators message. This is nullopt if no message has been
	/// received.
public: std::optional<msgs::Actuators> recvdActuatorsMsg;

	/// \brief Mutex to protect recvdActuatorsMsg.
public: std::mutex recvdActuatorsMsgMutex;

	/// \brief Polynomial for RPM to Thrust conversion - admits 5 degrees with asymmetric behavior
public: std::vector<double> positiveTorquePolynomial = {};
public: std::vector<double> positiveThrustPolynomial = {};
public: std::vector<double> negativeTorquePolynomial = {};
public: std::vector<double> negativeThrustPolynomial = {};

	/// \brief Gazebo communication node.
public: transport::Node node;
};

//////////////////////////////////////////////////
GenericMotorModel::GenericMotorModel()
	: dataPtr(std::make_unique<GenericMotorModelPrivate>())
{
}

//////////////////////////////////////////////////
void GenericMotorModel::Configure(const Entity &_entity,
				  const std::shared_ptr<const sdf::Element> &_sdf,
				  EntityComponentManager &_ecm,
				  EventManager &/*_eventMgr*/)
{
	this->dataPtr->model = Model(_entity);

	if (!this->dataPtr->model.Valid(_ecm)) {
		gzerr << "GenericMotorModel plugin should be attached to a model "
		      << "entity. Failed to initialize." << std::endl;
		return;
	}

	auto sdfClone = _sdf->Clone();

	this->dataPtr->robotNamespace.clear();

	if (sdfClone->HasElement("robotNamespace")) {
		this->dataPtr->robotNamespace =
			sdfClone->Get<std::string>("robotNamespace");

	} else {
		gzwarn << "No robotNamespace set using entity name.\n";
		this->dataPtr->robotNamespace = this->dataPtr->model.Name(_ecm);
	}

	// Get params from SDF
	if (sdfClone->HasElement("jointName")) {
		this->dataPtr->jointName = sdfClone->Get<std::string>("jointName");
	}

	if (this->dataPtr->jointName.empty()) {
		gzerr << "GenericMotorModel found an empty jointName parameter. "
		      << "Failed to initialize.";
		return;
	}

	if (sdfClone->HasElement("linkName")) {
		this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");
	}

	if (this->dataPtr->linkName.empty()) {
		gzerr << "GenericMotorModel found an empty linkName parameter. "
		      << "Failed to initialize.";
		return;
	}

	if (sdfClone->HasElement("actuator_number")) {
		this->dataPtr->actuatorNumber =
			sdfClone->GetElement("actuator_number")->Get<int>();

	} else if (sdfClone->HasElement("motorNumber")) {
		this->dataPtr->actuatorNumber =
			sdfClone->GetElement("motorNumber")->Get<int>();
		gzwarn << "<motorNumber> is depricated, "
		       << "please use <actuator_number>.\n";

	} else {
		gzerr << "Please specify a actuator_number.\n";
	}

	if (sdfClone->HasElement("turningDirection")) {
		auto turningDirection =
			sdfClone->GetElement("turningDirection")->Get<std::string>();

		if (turningDirection == "cw") {
			this->dataPtr->turningDirection = turning_direction::kCw;

		} else if (turningDirection == "ccw") {
			this->dataPtr->turningDirection = turning_direction::kCcw;

		} else {
			gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";
		}

	} else {
		gzerr << "Please specify a turning direction ('cw' or 'ccw').\n";
	}

	if (sdfClone->HasElement("controlMethod")) {
		auto controlMethod =
			sdfClone->GetElement("controlMethod")->Get<std::string>();

		if (controlMethod == "rpm") {
			this->dataPtr->controlMethod = ControlMethod::kRPM;

		} else if (controlMethod == "duty_cycle") {
			this->dataPtr->controlMethod = ControlMethod::kDutyCycle;

		} else {
			gzerr << "Please only use 'rpm' or 'duty_cycle' as controlMethod.\n";
		}

	} else {
		this->dataPtr->controlMethod = ControlMethod::kRPM;
	}

	// Check required parameters for each control method type
	if (this->dataPtr->controlMethod == ControlMethod::kDutyCycle) {
		if (sdfClone->HasElement("minCommand")) {
			this->dataPtr->minCommand =
				sdfClone->GetElement("minCommand")->Get<double>();

		} else {
			gzerr << "Please specify a minCommand.\n";
		}

		if (sdfClone->HasElement("maxDutyCycle")) {
			this->dataPtr->maxDutyCycle =
				sdfClone->GetElement("maxDutyCycle")->Get<double>();

		} else {
			gzerr << "Please specify a maxDutyCycle.\n";
		}

		if (sdfClone->HasElement("minDutyCycle")) {
			this->dataPtr->minDutyCycle =
				sdfClone->GetElement("minDutyCycle")->Get<double>();

		} else {
			gzerr << "Please specify a minDutyCycle.\n";
		}

	}

	if (sdfClone->HasElement("motorType")) {
		auto motorType = sdfClone->GetElement("motorType")->Get<std::string>();

		if (motorType == "velocity") {
			this->dataPtr->motorType = MotorType::kVelocity;

		} else if (motorType == "position") {
			this->dataPtr->motorType = MotorType::kPosition;
			gzerr << "motorType 'position' not implemented" << std::endl;

		} else if (motorType == "force") {
			this->dataPtr->motorType = MotorType::kForce;
			gzerr << "motorType 'force' not implemented" << std::endl;

		} else if (motorType == "force_polynomial") {
			this->dataPtr->motorType = MotorType::kForcePolynomial;

			// Check validity of required parameters
			if (sdfClone->HasElement("positiveThrustPolynomial")) {
				std::string raw_coeffs = sdfClone->GetElement("positiveThrustPolynomial")->Get<std::string>();
				std::vector<double> coeffs = this->ParsePolynomial(raw_coeffs);

				if (coeffs.size() < 4) {
					gzerr << "positiveThrustPolynomial must have at least 4 elements, but got "
					      << coeffs.size() << std::endl;
				}

				for (const auto &coeff : coeffs) {
					this->dataPtr->positiveThrustPolynomial.push_back(coeff);
				}

			} else {
				gzerr << "Please specify a positiveThrustPolynomial.\n";
			}

			if (sdfClone->HasElement("negativeThrustPolynomial")) {
				std::string raw_coeffs = sdfClone->GetElement("negativeThrustPolynomial")->Get<std::string>();
				std::vector<double> coeffs = this->ParsePolynomial(raw_coeffs);

				if (coeffs.size() < 4) {
					gzerr << "negativeThrustPolynomial must have at least 4 elements, but got "
					      << coeffs.size() << std::endl;
				}

				for (const auto &coeff : coeffs) {
					this->dataPtr->negativeThrustPolynomial.push_back(coeff);
				}

			} else {
				this->dataPtr->negativeThrustPolynomial = this->dataPtr->positiveThrustPolynomial;
				gzwarn << "Using positive thrust polynomial as negative thrust polynomial (symmetric performance)." << std::endl;
			}

			if (sdfClone->HasElement("positiveTorquePolynomial")) {
				std::string raw_coeffs = sdfClone->GetElement("positiveTorquePolynomial")->Get<std::string>();
				std::vector<double> coeffs = this->ParsePolynomial(raw_coeffs);

				if (coeffs.size() < 4 || coeffs.size() > 6) {
					gzerr << "positiveTorquePolynomial must have between 4 and 6 elements, but got "
					      << coeffs.size() << std::endl;
				}

				for (const auto &coeff : coeffs) {
					this->dataPtr->positiveTorquePolynomial.push_back(coeff);
				}

			} else {
				gzdbg << "No positive torque polynomial. Using zero.\n" << std::endl;
				this->dataPtr->positiveTorquePolynomial = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			}

			if (sdfClone->HasElement("negativeTorquePolynomial")) {
				std::string raw_coeffs = sdfClone->GetElement("negativeTorquePolynomial")->Get<std::string>();
				std::vector<double> coeffs = this->ParsePolynomial(raw_coeffs);

				if (coeffs.size() < 4 || coeffs.size() > 6) {
					gzerr << "negativeTorquePolynomial must have between 4 and 6 elements, but got "
					      << coeffs.size() << std::endl;
				}

				for (const auto &coeff : coeffs) {
					this->dataPtr->negativeTorquePolynomial.push_back(coeff);
				}

			} else {
				this->dataPtr->negativeTorquePolynomial = this->dataPtr->positiveTorquePolynomial;
				gzwarn << "Using positive torque polynomial as negative torque polynomial (symmetric performance)." << std::endl;
			}

			// print final polynomial values
			gzdbg << "Added motor " << this->dataPtr->actuatorNumber << ". Polynomials:\n - "
			      << "Positive thrust polynomial: ";

			for (const auto &val : this->dataPtr->positiveThrustPolynomial) {
				gzdbg << val << " ";
			}

			gzdbg << "\n - Negative thrust polynomial: ";

			for (const auto &val : this->dataPtr->negativeThrustPolynomial) {
				gzdbg << val << " ";
			}

			gzdbg << "\n - Positive torque polynomial: ";

			for (const auto &val : this->dataPtr->positiveTorquePolynomial) {
				gzdbg << val << " ";
			}

			gzdbg << "\n - Negative torque polynomial: ";

			for (const auto &val : this->dataPtr->negativeTorquePolynomial) {
				gzdbg << val << " ";
			}

			gzdbg << std::endl;

		} else {
			gzerr << "Please only use 'velocity', 'position', "
			      "'force' or 'force_polynomial' as motorType.\n"
			      << std::endl;
		}

	} else {
		gzwarn << "motorType not specified, using velocity.\n";
		this->dataPtr->motorType = MotorType::kVelocity;
	}

	sdfClone->Get<std::string>("commandSubTopic",
				   this->dataPtr->commandSubTopic, this->dataPtr->commandSubTopic);

	sdfClone->Get<double>("rotorDragCoefficient",
			      this->dataPtr->rotorDragCoefficient,
			      this->dataPtr->rotorDragCoefficient);
	sdfClone->Get<double>("rollingMomentCoefficient",
			      this->dataPtr->rollingMomentCoefficient,
			      this->dataPtr->rollingMomentCoefficient);
	sdfClone->Get<double>("maxRotVelocity",
			      this->dataPtr->maxRotVelocity, this->dataPtr->maxRotVelocity);
	sdfClone->Get<double>("motorConstant",
			      this->dataPtr->motorConstant, this->dataPtr->motorConstant);
	sdfClone->Get<double>("momentConstant",
			      this->dataPtr->momentConstant, this->dataPtr->momentConstant);
	sdfClone->Get<int>("bidirectionalMotor",
			   this->dataPtr->bidirectionalMotor, this->dataPtr->bidirectionalMotor);

	sdfClone->Get<double>("timeConstantUp",
			      this->dataPtr->timeConstantUp, this->dataPtr->timeConstantUp);
	sdfClone->Get<double>("timeConstantDown",
			      this->dataPtr->timeConstantDown, this->dataPtr->timeConstantDown);
	sdfClone->Get<double>("rotorVelocitySlowdownSim",
			      this->dataPtr->rotorVelocitySlowdownSim, 10);

	// Create the first order filter.
	this->dataPtr->rotorVelocityFilter =
		std::make_unique<FirstOrderFilter<double>>(
			this->dataPtr->timeConstantUp, this->dataPtr->timeConstantDown,
			this->dataPtr->refMotorInput);

	// Subscribe to actuator command messages
	std::string topic = transport::TopicUtils::AsValidTopic(
				    this->dataPtr->robotNamespace + "/" + this->dataPtr->commandSubTopic);

	if (topic.empty()) {
		gzerr << "Failed to create topic for [" << this->dataPtr->robotNamespace
		      << "]" << std::endl;
		return;

	} else {
		gzdbg << "Listening to topic: " << topic << std::endl;
	}

	this->dataPtr->node.Subscribe(topic,
				      &GenericMotorModelPrivate::OnActuatorMsg, this->dataPtr.get());
}

//////////////////////////////////////////////////
void GenericMotorModel::PreUpdate(const UpdateInfo &_info,
				  EntityComponentManager &_ecm)
{
	GZ_PROFILE("GenericMotorModel::PreUpdate");

	// \TODO(anyone) Support rewind
	if (_info.dt < std::chrono::steady_clock::duration::zero()) {
		gzwarn << "Detected jump back in time ["
		       << std::chrono::duration<double>(_info.dt).count()
		       << "s]. System may not work properly." << std::endl;
	}

	// If the joint or links haven't been identified yet, look for them
	if (this->dataPtr->jointEntity == kNullEntity) {
		this->dataPtr->jointEntity =
			this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);

		const auto parentLinkName = _ecm.Component<components::ParentLinkName>(
						    this->dataPtr->jointEntity);

		if (parentLinkName) {
			this->dataPtr->parentLinkName = parentLinkName->Data();
		}
	}

	if (this->dataPtr->linkEntity == kNullEntity) {
		this->dataPtr->linkEntity =
			this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
	}

	if (this->dataPtr->parentLinkEntity == kNullEntity) {
		this->dataPtr->parentLinkEntity =
			this->dataPtr->model.LinkByName(_ecm, this->dataPtr->parentLinkName);
	}

	if (this->dataPtr->jointEntity == kNullEntity ||
	    this->dataPtr->linkEntity == kNullEntity ||
	    this->dataPtr->parentLinkEntity == kNullEntity) {
		return;
	}

	// skip UpdateForcesAndMoments if needed components are missing
	bool doUpdateForcesAndMoments = true;

	const auto jointVelocity = _ecm.Component<components::JointVelocity>(
					   this->dataPtr->jointEntity);

	if (!jointVelocity) {
		_ecm.CreateComponent(this->dataPtr->jointEntity,
				     components::JointVelocity());
		doUpdateForcesAndMoments = false;

	} else if (jointVelocity->Data().empty()) {
		doUpdateForcesAndMoments = false;
	}

	if (!_ecm.Component<components::JointVelocityCmd>(
		    this->dataPtr->jointEntity)) {
		_ecm.CreateComponent(this->dataPtr->jointEntity,
				     components::JointVelocityCmd({0}));
		doUpdateForcesAndMoments = false;
	}

	if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity)) {
		_ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
		doUpdateForcesAndMoments = false;
	}

	if (!_ecm.Component<components::WorldLinearVelocity>(
		    this->dataPtr->linkEntity)) {
		_ecm.CreateComponent(this->dataPtr->linkEntity,
				     components::WorldLinearVelocity());
		doUpdateForcesAndMoments = false;
	}

	if (!_ecm.Component<components::WorldPose>(this->dataPtr->parentLinkEntity)) {
		_ecm.CreateComponent(this->dataPtr->parentLinkEntity,
				     components::WorldPose());
		doUpdateForcesAndMoments = false;
	}

	// Nothing left to do if paused.
	if (_info.paused) {
		return;
	}

	this->dataPtr->samplingTime =
		std::chrono::duration<double>(_info.dt).count();

	if (doUpdateForcesAndMoments) {
		this->dataPtr->UpdateForcesAndMoments(_ecm);
	}
}

//////////////////////////////////////////////////
void GenericMotorModelPrivate::OnActuatorMsg(
	const msgs::Actuators &_msg)
{
	std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
	this->recvdActuatorsMsg = _msg;
}

//////////////////////////////////////////////////
void GenericMotorModelPrivate::UpdateForcesAndMoments(
	EntityComponentManager &_ecm)
{
	GZ_PROFILE("GenericMotorModelPrivate::UpdateForcesAndMoments");

	std::optional<msgs::Actuators> msg;
	auto actuatorMsgComp =
		_ecm.Component<components::Actuators>(this->model.Entity());

	// Actuators messages can come in from transport or via a component. If a
	// component is available, it takes precedence.
	if (actuatorMsgComp) {
		msg = actuatorMsgComp->Data();

	} else {
		std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);

		if (this->recvdActuatorsMsg.has_value()) {
			msg = *this->recvdActuatorsMsg;
			this->recvdActuatorsMsg.reset();
		}
	}

	if (msg.has_value()) {
		if (this->actuatorNumber > msg->velocity_size() - 1) {
			gzerr << "You tried to access index " << this->actuatorNumber
			      << " of the Actuator array which is of size "
			      << std::max(msg->velocity_size(), msg->normalized_size()) << std::endl;
			return;
		}

		if (this->motorType == MotorType::kVelocity) {
			this->refMotorInput = std::min(
						      static_cast<double>(msg->velocity(this->actuatorNumber)),
						      static_cast<double>(this->maxRotVelocity));

		} else if (this->motorType == MotorType::kForcePolynomial &&
			   this->controlMethod == ControlMethod::kRPM) {
			// Uses RPM as reference
			this->refMotorInput = std::min(
						      static_cast<double>(msg->velocity(this->actuatorNumber)),
						      static_cast<double>(this->maxRotVelocity));

		} else if (this->motorType == MotorType::kForcePolynomial &&
			   this->controlMethod == ControlMethod::kDutyCycle &&
			   msg->velocity(this->actuatorNumber) >= this->minDutyCycle) {
			// Create normalized reference based on the duty cycle
			this->refMotorInput = ((msg->velocity(this->actuatorNumber) - (this->minDutyCycle +
						(this->maxDutyCycle - this->minDutyCycle) * this->bidirectionalMotor / 2.0)) / ((this->maxDutyCycle -
								this->minDutyCycle) - (this->maxDutyCycle - this->minDutyCycle) * this->bidirectionalMotor / 2.0));

		} else {
			this->refMotorInput = msg->velocity(this->actuatorNumber);
		}
	}

	switch (this->motorType) {
	case (MotorType::kPosition): {
			// TODO: Integrate
			// double err = joint_->GetAngle(0).Radian() - this->refMotorInput;
			// double force = pids_.Update(err, this->samplingTime);
			// joint_->SetForce(0, force);
			break;
		}

	case (MotorType::kForce): {
			// TODO: Integrate
			// joint_->SetForce(0, this->refMotorInput);
			break;
		}

	case (MotorType::kForcePolynomial): {
			sim::Link link(this->linkEntity);
			const auto worldPose = link.WorldPose(_ecm);
			using Vector3 = math::Vector3d;

			// Compute thrust
			double thrust = 0.0;

			if (this->refMotorInput >= this->minCommand) {
				for (unsigned int i = 0; i < this->positiveThrustPolynomial.size(); i++) {
					thrust += this->positiveThrustPolynomial[i] * std::pow(this->refMotorInput, i);
				}

			} else if (this->refMotorInput <= -this->minCommand) {
				for (unsigned int i = 0; i < this->negativeThrustPolynomial.size(); i++) {
					thrust -= this->negativeThrustPolynomial[i] * std::pow(std::abs(this->refMotorInput), i);
				}

			} else {
				thrust = 0.0;
			}

			link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));

			// Compute torques
			double torque = 0.0;

			if (this->refMotorInput >= this->minCommand) {
				for (unsigned int i = 0; i < this->positiveTorquePolynomial.size(); i++) {
					torque += this->positiveTorquePolynomial[i] * std::pow(this->refMotorInput, i);
				}

			} else if (this->refMotorInput <= -this->minCommand) {
				for (unsigned int i = 0; i < this->negativeTorquePolynomial.size(); i++) {
					torque -= this->negativeTorquePolynomial[i] * std::pow(std::abs(this->refMotorInput), i);
				}

			} else {
				torque = 0.0;
			}

			link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, torque)));

			// Set joint velocities
			const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
							 this->jointEntity);
			*jointVelCmd = components::JointVelocityCmd({this->refMotorInput / this->rotorVelocitySlowdownSim});
			break;
		}

	default: { // MotorType::kVelocity
			const auto jointVelocity = _ecm.Component<components::JointVelocity>(
							   this->jointEntity);
			double motorRotVel = jointVelocity->Data()[0];

			if (motorRotVel / (2 * GZ_PI) > 1 / (2 * this->samplingTime)) {
				gzerr << "Aliasing on motor [" << this->actuatorNumber
				      << "] might occur. Consider making smaller simulation time "
				      "steps or raising the rotorVelocitySlowdownSim param.\n";
			}

			double realMotorVelocity =
				motorRotVel * this->rotorVelocitySlowdownSim;
			// Get the direction of the rotor rotation.
			int realMotorVelocitySign =
				(realMotorVelocity > 0) - (realMotorVelocity < 0);
			// Assuming symmetric propellers (or rotors) for the thrust calculation.
			double thrust = this->turningDirection * realMotorVelocitySign *
					realMotorVelocity * realMotorVelocity *
					this->motorConstant;

			using Pose = math::Pose3d;
			using Vector3 = math::Vector3d;

			Link link(this->linkEntity);
			const auto worldPose = link.WorldPose(_ecm);

			// Apply a force to the link.
			link.AddWorldForce(_ecm,
					   worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));

			const auto jointPose = _ecm.Component<components::Pose>(
						       this->jointEntity);

			if (!jointPose) {
				gzerr << "joint " << this->jointName << " has no Pose"
				      << "component" << std::endl;
				return;
			}

			// computer joint world pose by multiplying child link WorldPose
			// with joint Pose
			Pose jointWorldPose = *worldPose * jointPose->Data();

			const auto jointAxisComp = _ecm.Component<components::JointAxis>(
							   this->jointEntity);

			if (!jointAxisComp) {
				gzerr << "joint " << this->jointName << " has no JointAxis"
				      << "component" << std::endl;
				return;
			}

			const auto worldLinearVel = link.WorldLinearVelocity(_ecm);

			Entity windEntity = _ecm.EntityByComponents(components::Wind());
			auto windLinearVel =
				_ecm.Component<components::WorldLinearVelocity>(windEntity);
			Vector3 windSpeedWorld = windLinearVel->Data();

			// Forces from Philppe Martin's and Erwan Salaun's
			// 2010 IEEE Conference on Robotics and Automation paper
			// The True Role of Accelerometer Feedback in Quadrotor Control
			// - \omega * \lambda_1 * V_A^{\perp}
			Vector3 jointAxis =
				jointWorldPose.Rot().RotateVector(jointAxisComp->Data().Xyz());
			Vector3 bodyVelocityWorld = *worldLinearVel;
			Vector3 relativeWindVelocityWorld = bodyVelocityWorld - windSpeedWorld;
			Vector3 bodyVelocityPerpendicular =
				relativeWindVelocityWorld -
				(relativeWindVelocityWorld.Dot(jointAxis) * jointAxis);
			Vector3 airDrag = -std::abs(realMotorVelocity) *
					  this->rotorDragCoefficient *
					  bodyVelocityPerpendicular;

			// Apply air drag to link.
			link.AddWorldForce(_ecm, airDrag);
			// Moments get the parent link, such that the resulting torques can be
			// applied.
			Vector3 parentWorldTorque;
			auto parentWrenchComp =
				_ecm.Component<components::ExternalWorldWrenchCmd>(
					this->parentLinkEntity);
			// gazebo_motor_model.cpp subtracts the GetWorldCoGPose() of the
			// child link from the parent but only uses the rotation component.
			// Since GetWorldCoGPose() uses the link frame orientation, it
			// is equivalent to use WorldPose().Rot().
			Link parentLink(this->parentLinkEntity);
			const auto parentWorldPose = parentLink.WorldPose(_ecm);
			// The transformation from the parent_link to the link_.
			// Pose poseDifference =
			//  parent_links.at(0)->GetWorldCoGPose().Inverse()
			//  * link_->GetWorldCoGPose()
			Pose poseDifference = (*parentWorldPose).Inverse() * (*worldPose);
			Vector3 dragTorque(
				0, 0, -this->turningDirection * thrust * this->momentConstant);
			// Transforming the drag torque into the parent frame to handle
			// arbitrary rotor orientations.
			Vector3 dragTorqueParentFrame =
				poseDifference.Rot().RotateVector(dragTorque);
			parentWorldTorque =
				parentWorldPose->Rot().RotateVector(dragTorqueParentFrame);

			Vector3 rollingMoment;
			// - \omega * \mu_1 * V_A^{\perp}
			rollingMoment = -std::abs(realMotorVelocity) *
					this->rollingMomentCoefficient *
					bodyVelocityPerpendicular;
			parentWorldTorque += rollingMoment;

			if (!parentWrenchComp) {
				components::ExternalWorldWrenchCmd wrench;
				msgs::Set(wrench.Data().mutable_torque(), parentWorldTorque);
				_ecm.CreateComponent(this->parentLinkEntity, wrench);

			} else {
				msgs::Set(parentWrenchComp->Data().mutable_torque(),
					  msgs::Convert(parentWrenchComp->Data().torque()) + parentWorldTorque);
			}

			// Apply the filter on the motor's velocity.
			double refMotorRotVel;
			refMotorRotVel = this->rotorVelocityFilter->UpdateFilter(
						 this->refMotorInput, this->samplingTime);

			const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
							 this->jointEntity);
			*jointVelCmd = components::JointVelocityCmd({
				this->turningDirection *refMotorRotVel
				/ this->rotorVelocitySlowdownSim});
		}
	}
}

GZ_ADD_PLUGIN(GenericMotorModel,
	      System,
	      GenericMotorModel::ISystemConfigure,
	      GenericMotorModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(GenericMotorModel,
		    "gz::sim::systems::GenericMotorModel")
