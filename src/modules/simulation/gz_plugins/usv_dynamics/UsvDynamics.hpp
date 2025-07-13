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

#pragma once

#include <Eigen/Core>
#include <string>
#include <chrono>
#include <unordered_map>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/Element.hh>

#define GRAVITY 9.815

namespace custom
{
	/// \brief Plugin class to implement hydrodynamics and wave response for USV.
	/// This plugin accepts the following SDF parameters:
	///
	/// <bodyName>: Name of base link for receiving pose and applying forces.
	/// <boatLength>: Boat length [m]. Default value is 1.35.
	/// <boatWidth>: Boat width [m]. Default value is 1.
	/// <hullRadius>: Demi-hull radius [m]. Default value is 0.213.
	/// <waterDensity>: Water density [kg/m^3]. Default value is 997.7735.
	/// <waterLevel>: Water height [m]. Default value is 0.5.
	/// <xDotU>: Added mass coeff, surge. Default value is 5.
	/// <yDotV>: Added mass coeff, sway. Default value is 5.
	/// <nDotR>: Added mass coeff, yaw. Default value is 1.
	/// <xU>: Linear drag coeff surge. Default value is 20.
	/// <xUU>: Quadratic drag coeff surge. Default value is 0.
	/// <yV>: Linear drag coeff sway. Default value is 20.
	/// <yVV>: Quadratic drag coeff sway. Default value is 0.
	/// <zW>: Linear drag coeff heave. Default value is 20.
	/// <kP>: Linear drag coeff roll. Default value is 20.
	/// <mQ>: Linear drag coeff pitch. Default value is 20.
	/// <nR>: Linear drag coeff yaw. Default value is 20.
	/// <nRR>: Quadratic drag coeff yaw. Default value is 0.
	/// <length_n>: Number of length segments for buoyancy calculation.
	class UsvDynamics :
		public gz::sim::System,
		public gz::sim::ISystemConfigure,
		public gz::sim::ISystemPreUpdate
	{
	public:
		/// \brief Constructor.
		UsvDynamics();

		/// \brief Destructor.
		virtual ~UsvDynamics() = default;

		// Documentation inherited.
		void Configure(const gz::sim::Entity &_modelEntity,
			       const std::shared_ptr<const sdf::Element> &_sdf,
			       gz::sim::EntityComponentManager &_ecm,
			       gz::sim::EventManager &_eventMgr) override;

		/// \brief Callback for Gazebo simulation engine.
		void PreUpdate(const gz::sim::UpdateInfo &_info,
			       gz::sim::EntityComponentManager &_ecm) override;

	private:
		/// \brief Structure to hold USV configuration for a model
		struct UsvModelData {
			gz::sim::Entity modelEntity;
			gz::sim::Entity linkEntity;
			gz::sim::Model model{gz::sim::kNullEntity};
			gz::sim::Link link{gz::sim::kNullEntity};
			
			// USV parameters
			double waterLevel;
			double waterDensity;
			double paramXdotU, paramYdotV, paramNdotR;
			double paramXu, paramXuu, paramYv, paramYvv, paramZw;
			double paramKp, paramMq, paramNr, paramNrr;
			double paramBoatLength, paramBoatWidth, paramHullRadius;
			int paramLengthN;
			Eigen::MatrixXd Ma;
			
			// State tracking
			gz::math::Vector3d prevLinVel{0, 0, 0};
			gz::math::Vector3d prevAngVel{0, 0, 0};
			bool firstUpdate{true};
		};

		/// \brief USV data for this model instance
		UsvModelData _usvData;

		/// \brief Convenience function for calculating the area of circle segment
		/// \param[in] R Radius of circle
		/// \param[in] h Height of the chord line
		/// \return The area
		double CircleSegment(double R, double h);

		/// \brief Simulation time of the last update.
		std::chrono::steady_clock::duration _prevUpdateTime{0};

		/// \brief First update flag to initialize timing.
		bool _firstUpdate{true};
	};
} // end namespace custom