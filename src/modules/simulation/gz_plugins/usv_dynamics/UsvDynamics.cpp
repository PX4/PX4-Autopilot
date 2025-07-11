/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include <cmath>
#include <functional>
#include <sstream>
#include <algorithm>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/msgs/wrench.pb.h>

#include "UsvDynamics.hpp"

using namespace custom;

#define GRAVITY 9.815

//////////////////////////////////////////////////
UsvDynamics::UsvDynamics() : _firstUpdate(true)
{
}

//////////////////////////////////////////////////
void UsvDynamics::Configure(const gz::sim::Entity &_modelEntity,
			    const std::shared_ptr<const sdf::Element> &_sdf,
			    gz::sim::EntityComponentManager &_ecm,
			    gz::sim::EventManager &)
{
	gzdbg << "Loading USV Dynamics Model Plugin" << std::endl;

	// Store the model entity
	this->_usvData.modelEntity = _modelEntity;
	this->_usvData.model = gz::sim::Model(_modelEntity);

	// Get link name from plugin SDF
	std::string linkName = _sdf->Get<std::string>("bodyName", "base_link").first;
	this->_usvData.linkEntity = this->_usvData.model.LinkByName(_ecm, linkName);

	if (this->_usvData.linkEntity == gz::sim::kNullEntity) {
		gzerr << "USV dynamics plugin error: bodyName: " << linkName << " does not exist in model" << std::endl;
		return;
	}

	this->_usvData.link = gz::sim::Link(this->_usvData.linkEntity);
	this->_usvData.link.EnableVelocityChecks(_ecm, true);

	// Load parameters from plugin configuration
	this->_usvData.waterLevel = _sdf->Get<double>("waterLevel", 0.0).first;
	this->_usvData.waterDensity = _sdf->Get<double>("waterDensity", 997.7735).first;
	this->_usvData.paramXdotU = _sdf->Get<double>("xDotU", 50).first;
	this->_usvData.paramYdotV = _sdf->Get<double>("yDotV", 50).first;
	this->_usvData.paramNdotR = _sdf->Get<double>("nDotR", 15).first;
	this->_usvData.paramXu = _sdf->Get<double>("xU", 30).first;
	this->_usvData.paramXuu = _sdf->Get<double>("xUU", 10).first;
	this->_usvData.paramYv = _sdf->Get<double>("yV", 40).first;
	this->_usvData.paramYvv = _sdf->Get<double>("yVV", 20).first;
	this->_usvData.paramZw = _sdf->Get<double>("zW", 50).first;
	this->_usvData.paramKp = _sdf->Get<double>("kP", 25).first;
	this->_usvData.paramMq = _sdf->Get<double>("mQ", 25).first;
	this->_usvData.paramNr = _sdf->Get<double>("nR", 35).first;
	this->_usvData.paramNrr = _sdf->Get<double>("nRR", 5).first;
	this->_usvData.paramHullRadius = _sdf->Get<double>("hullRadius", 0.2).first;
	this->_usvData.paramBoatWidth = _sdf->Get<double>("boatWidth", 2.06).first;
	this->_usvData.paramBoatLength = _sdf->Get<double>("boatLength", 4.7).first;
	this->_usvData.paramLengthN = _sdf->Get<int>("length_n", 15).first;

	// Initialize Added Mass Matrix
	this->_usvData.Ma = Eigen::MatrixXd(6, 6);
	this->_usvData.Ma <<
		this->_usvData.paramXdotU, 0,                0,   0,   0,   0,
		0,                this->_usvData.paramYdotV, 0,   0,   0,   0,
		0,                0,                0.1, 0,   0,   0,
		0,                0,                0,   0.1, 0,   0,
		0,                0,                0,   0,   0.1, 0,
		0,                0,                0,   0,   0,   this->_usvData.paramNdotR;

	gzdbg << "USV Dynamics configured for model with link: " << linkName << std::endl;
}

//////////////////////////////////////////////////
double UsvDynamics::CircleSegment(double R, double h)
{
	return R * R * acos((R - h) / R) - (R - h) * sqrt(2 * R * h - h * h);
}

//////////////////////////////////////////////////
void UsvDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
			    gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	// Check if plugin was properly configured
	if (this->_usvData.linkEntity == gz::sim::kNullEntity) {
		return; // Plugin not properly configured, skip
	}

	// Initialize timing on first update
	if (this->_firstUpdate) {
		this->_prevUpdateTime = _info.simTime;
		this->_firstUpdate = false;
		return;
	}

	// Calculate time step
	auto dt_duration = _info.simTime - this->_prevUpdateTime;
	double dt = std::chrono::duration<double>(dt_duration).count();
	this->_prevUpdateTime = _info.simTime;

	if (dt <= 0.0) {
		return; // Skip if no time has passed
	}

	// Handle first update for this model
	if (this->_usvData.firstUpdate) {
		this->_usvData.firstUpdate = false;
		return;
	}

	// Get Pose/Orientation from Gazebo using the link's world pose
	auto worldPose = this->_usvData.link.WorldPose(_ecm);
	if (!worldPose) {
		return; // Skip if pose not available
	}
	const gz::math::Pose3d kPose = worldPose.value();

	// Get body-centered linear and angular velocities using Link methods
	auto worldLinVel = this->_usvData.link.WorldLinearVelocity(_ecm);
	auto worldAngVel = this->_usvData.link.WorldAngularVelocity(_ecm);

	// Use zero velocity if not available yet (during startup)
	gz::math::Vector3d kVelLinearWorld(0, 0, 0);
	gz::math::Vector3d kVelAngularWorld(0, 0, 0);

	if (worldLinVel) {
		kVelLinearWorld = worldLinVel.value();
	}
	if (worldAngVel) {
		kVelAngularWorld = worldAngVel.value();
	}

	// Transform to body frame
	const gz::math::Vector3d kVelLinearBody = kPose.Rot().Inverse().RotateVector(kVelLinearWorld);
	const gz::math::Vector3d kVelAngularBody = kPose.Rot().Inverse().RotateVector(kVelAngularWorld);

	// Estimate the linear and angular accelerations
	const gz::math::Vector3d kAccelLinearBody = (kVelLinearBody - this->_usvData.prevLinVel) / dt;
	this->_usvData.prevLinVel = kVelLinearBody;

	const gz::math::Vector3d kAccelAngularBody = (kVelAngularBody - this->_usvData.prevAngVel) / dt;
	this->_usvData.prevAngVel = kVelAngularBody;

	// Create state and derivative of state (accelerations)
	Eigen::VectorXd stateDot = Eigen::VectorXd(6);
	Eigen::VectorXd state = Eigen::VectorXd(6);
	Eigen::MatrixXd Cmat = Eigen::MatrixXd::Zero(6, 6);
	Eigen::MatrixXd Dmat = Eigen::MatrixXd::Zero(6, 6);

	stateDot << kAccelLinearBody.X(), kAccelLinearBody.Y(), kAccelLinearBody.Z(),
		kAccelAngularBody.X(), kAccelAngularBody.Y(), kAccelAngularBody.Z();

	state << kVelLinearBody.X(), kVelLinearBody.Y(), kVelLinearBody.Z(),
		kVelAngularBody.X(), kVelAngularBody.Y(), kVelAngularBody.Z();

	// Added Mass
	const Eigen::VectorXd kAmassVec = -1.0 * this->_usvData.Ma * stateDot;

	// Coriolis - added mass components
	Cmat(0, 5) = this->_usvData.paramYdotV * kVelLinearBody.Y();
	Cmat(1, 5) = this->_usvData.paramXdotU * kVelLinearBody.X();
	Cmat(5, 0) = this->_usvData.paramYdotV * kVelLinearBody.Y();
	Cmat(5, 1) = this->_usvData.paramXdotU * kVelLinearBody.X();

	// Drag
	Dmat(0, 0) = this->_usvData.paramXu + this->_usvData.paramXuu * std::abs(kVelLinearBody.X());
	Dmat(1, 1) = this->_usvData.paramYv + this->_usvData.paramYvv * std::abs(kVelLinearBody.Y());
	Dmat(2, 2) = this->_usvData.paramZw;
	Dmat(3, 3) = this->_usvData.paramKp;
	Dmat(4, 4) = this->_usvData.paramMq;
	Dmat(5, 5) = this->_usvData.paramNr + this->_usvData.paramNrr * std::abs(kVelAngularBody.Z());

	const Eigen::VectorXd kDvec = -1.0 * Dmat * state;

	// Sum all forces - in body frame
	const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

	// Initialize total force and torque (in body frame like classic plugin)
	gz::math::Vector3d totalForce(kForceSum(0), kForceSum(1), kForceSum(2));
	gz::math::Vector3d totalTorque(kForceSum(3), kForceSum(4), kForceSum(5));

	// Loop over boat grid points for buoyancy calculation
	gz::math::Vector3d bpnt(0, 0, 0);
	gz::math::Vector3d bpntW(0, 0, 0);

	// For each hull
	for (int ii = 0; ii < 2; ii++) {
		// Grid point in boat frame
		bpnt.Y((ii * 2.0 - 1.0) * this->_usvData.paramBoatWidth / 2.0);

		// For each length segment
		for (int jj = 1; jj <= this->_usvData.paramLengthN; jj++) {
			bpnt.X(((jj - 0.5) / (static_cast<double>(this->_usvData.paramLengthN)) - 0.5) *
			       this->_usvData.paramBoatLength);

			// Transform from vessel to water/world frame
			bpntW = kPose.Rot().RotateVector(bpnt);

			// Vertical location of boat grid point in world frame
			const double kDdz = kPose.Pos().Z() + bpntW.Z();

			// Find vertical displacement of wave field
			gz::math::Vector3d X;
			X.X() = kPose.Pos().X() + bpntW.X();
			X.Y() = kPose.Pos().Y() + bpntW.Y();

			// Compute the depth at the grid point (no waves for now)
			double depth = 0.0;

			// Vertical wave displacement
			double dz = depth + X.Z();

			// Total z location of boat grid point relative to water surface
			double deltaZ = (this->_usvData.waterLevel + dz) - kDdz;
			deltaZ = std::max(deltaZ, 0.0);  // enforce only upward buoy force
			deltaZ = std::min(deltaZ, this->_usvData.paramHullRadius);

			// Buoyancy force at grid point
			const double kBuoyForce = CircleSegment(this->_usvData.paramHullRadius, deltaZ) *
						  this->_usvData.paramBoatLength / (static_cast<double>(this->_usvData.paramLengthN)) *
						  GRAVITY * this->_usvData.waterDensity;

			// Apply buoyancy force at grid point location (like classic plugin)
			// Force is vertical (world frame), but we apply it at the grid point location
			gz::math::Vector3d forceAtPoint(0, 0, kBuoyForce);
			
			// In classic plugin, forces are applied relative to link frame
			// AddForceAtPosition effectively does: force + position.Cross(force) for torque
			// We need to transform the vertical force to body frame and add torque manually
			gz::math::Vector3d forceInBody = kPose.Rot().Inverse().RotateVector(forceAtPoint);
			gz::math::Vector3d torqueFromForce = bpnt.Cross(forceInBody);
			
			totalForce += forceInBody;
			totalTorque += torqueFromForce;
		}
	}

	// Apply forces and torques in body frame (like classic plugin AddRelativeForce/Torque)
	// Transform to world frame for modern Gazebo
	gz::math::Vector3d forceWorld = kPose.Rot().RotateVector(totalForce);
	gz::math::Vector3d torqueWorld = kPose.Rot().RotateVector(totalTorque);

	// Apply forces and torques to the link
	gz::msgs::Wrench wrenchMsg;
	gz::msgs::Set(wrenchMsg.mutable_force(), forceWorld);
	gz::msgs::Set(wrenchMsg.mutable_torque(), torqueWorld);

	// Create or update the external wrench component
	_ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(this->_usvData.linkEntity, {wrenchMsg});
}

// Register the plugin
GZ_ADD_PLUGIN(UsvDynamics,
	      gz::sim::System,
	      UsvDynamics::ISystemConfigure,
	      UsvDynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(UsvDynamics, "custom::UsvDynamics")