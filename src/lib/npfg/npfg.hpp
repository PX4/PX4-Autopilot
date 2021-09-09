/****************************************************************************
 *
 * Copyright (c) 2021 Autonomous Systems Lab, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/*
 * @file npfg.hpp
 * Implementation of a lateral-directional nonlinear path following guidance
 * law with excess wind handling.
 *
 * Acknowledgements and References:
 *
 * TODO
 *
 */

#ifndef NPFG_H_
#define NPFG_H_

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

/*
 * NPFG
 * Lateral-directional nonlinear path following guidance logic with excess wind handling
 */
class NPFG
{

public:

	/*
	 * Set the nominal controller period [s].
	 */
	void setPeriod(float period) { period_ = math::max(period, EPSILON); }

	/*
	 * Set the nominal controller damping ratio.
	 */
	void setDamping(float damping) { damping_ = math::constrain(damping, EPSILON, 1.0f); }

	/*
	 * Enable automatic lower bounding of the user set controller period.
	 */
	void enablePeriodLB(const bool en) { en_period_lb_ = en; }

	/*
	 * Enable automatic adaptation of the user set controller period for track keeping
	 * performance.
	 */
	void enablePeriodUB(const bool en) { en_period_ub_ = en; }

	/*
	 * Ramp in any automatic period adaptations with the track proximity.
	 */
	void rampInAdaptedPeriod(const bool en) { ramp_in_adapted_period_ = en; }

	/*
	 * Enable minimum forward ground speed maintenance logic.
	 */
	void enableMinGroundSpeed(const bool en) { en_min_ground_speed_ = en; }

	/*
	 * Enable track keeping logic in excess wind conditions.
	 */
	void enableTrackKeeping(const bool en) { en_track_keeping_ = en; }

	/*
	 * Enable wind excess regulation. Disabling this param disables all airspeed
	 * reference incrementaion (airspeed reference will always be nominal).
	 */
	void enableWindExcessRegulation(const bool en) { en_wind_excess_regulation_ = en; }

	/*
	 * Set the minimum allowed forward ground speed [m/s].
	 */
	void setMinGroundSpeed(float min_gsp) { min_gsp_cmd_ = math::max(min_gsp, 0.0f); }

	/*
	 * Set the maximum value of the minimum forward ground speed command for track
	 * keeping (occurs at the track error boundary) [m/s].
	 */
	void setMaxTrackKeepingMinGroundSpeed(float min_gsp) { min_gsp_track_keeping_max_ = math::max(min_gsp, 0.0f); }

	/*
	 * Set the normalized track error fraction.
	 */
	void setNormalizedTrackErrorFraction(float nte_fraction) { inv_nte_fraction_ = 1.0f / math::max(nte_fraction, 0.1f); }

	/*
	 * Set the nominal airspeed reference [m/s].
	 */
	void setAirspeedNom(float airsp) { airspeed_nom_ = math::max(airsp, 0.1f); }

	/*
	 * Set the maximum airspeed reference [m/s].
	 */
	void setAirspeedMax(float airsp) { airspeed_max_ = math::max(airsp, 0.1f); }

	/*
	 * Set the autopilot roll response time constant [s].
	 */
	void setRollTimeConst(float tc) { roll_time_const_ = math::max(tc, 0.1f); }

	/*
	 * Set the airspeed buffer size.
	 */
	void setAirspeedBuffer(float buf) { airspeed_buffer_ = math::max(buf, 0.1f); }

	/*
	 * @return Controller proportional gain [rad/s]
	 */
	float getPGain() const { return p_gain_; }

	/*
	 * @return Controller time constant [s]
	 */
	float getTimeConst() const { return time_const_; }

	/*
	 * @return Adapted controller period [s]
	 */
	float getAdaptedPeriod() const { return adapted_period_; }

	/*
	 * @return Track error boundary [m]
	 */
	float getTrackErrorBound() const { return track_error_bound_; }

	/*
	 * @return Signed track error [m]
	 */
	float getTrackError() const { return signed_track_error_; }

	/*
	 * @return Airspeed reference [m/s]
	 */
	float getAirspeedRef() const { return airspeed_ref_; }

	/*
	 * @return Heading angle reference [rad]
	 */
	float getHeadingRef() const { return atan2f(air_vel_ref_(1), air_vel_ref_(0)); }

	/*
	 * @return Bearing angle [rad]
	 */
	float getBearing() const { return atan2f(bearing_vec_(1), bearing_vec_(0)); }

	/*
	 * @return Lateral acceleration command [m/s^2]
	 */
	float getLateralAccel() const { return lateral_accel_; }

	/*
	 * @return Feed-forward lateral acceleration command increment for tracking
	 * path curvature [m/s^2]
	 */
	float getLateralAccelFF() const { return lateral_accel_ff_; }

	/*
	 * @return Bearing feasibility [0, 1]
	 */
	float getBearingFeas() const { return feas_; }

	/*
	 * @return On-track bearing feasibility [0, 1]
	 */
	float getOnTrackBearingFeas() const { return feas_on_track_; }

	/*
	 * @return Minimum forward ground speed reference [m/s]
	 */
	float getMinGroundSpeedRef() const { return min_ground_speed_ref_; }

	/*******************************************************************************
	 * PX4 NAVIGATION INTERFACE FUNCTIONS (provide similar functionality to ECL_L1_Pos_Controller)
	 */

	/*
	 * Waypoint handling logic following closely to the ECL_L1_Pos_Controller
	 * method of the same name. Takes two waypoints and determines the relevant
	 * parameters for evaluating the NPFG guidance law, then updates control setpoints.
	 *
	 * @param[in] waypoint_A Waypoint A (segment start) position in WGS84 coordinates
	 *            (lat,lon) [deg]
	 * @param[in] waypoint_B Waypoint B (segment end) position in WGS84 coordinates
	 *            (lat,lon) [deg]
	 * @param[in] vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateWaypoints(const matrix::Vector2d &waypoint_A, const matrix::Vector2d &waypoint_B,
			       const matrix::Vector2d &vehicle_pos, const matrix::Vector2f &ground_vel,
			       const matrix::Vector2f &wind_vel);

	/*
	 * Loitering (unlimited) logic. Takes loiter center, radius, and direction and
	 * determines the relevant parameters for evaluating the NPFG guidance law,
	 * then updates control setpoints.
	 *
	 * @param[in] loiter_center The position of the center of the loiter circle [m]
	 * @param[in] vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] radius Loiter radius [m]
	 * @param[in] loiter_direction Loiter direction: -1=counter-clockwise, 1=clockwise
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateLoiter(const matrix::Vector2d &loiter_center, const matrix::Vector2d &vehicle_pos,
			    float radius, int8_t loiter_direction, const matrix::Vector2f &ground_vel,
			    const matrix::Vector2f &wind_vel);

	/*
	 * Navigate on a fixed heading.
	 *
	 * This only holds a certain (air mass relative) direction and does not perform
	 * cross track correction. Helpful for semi-autonomous modes. Introduced
	 * by in ECL_L1_Pos_Controller, augmented for use with NPFG here.
	 *
	 * @param[in] heading_ref Reference heading angle [rad]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateHeading(float heading_ref, const matrix::Vector2f &ground_vel,
			     const matrix::Vector2f &wind_vel);

	/*
	 * Navigate on a fixed bearing.
	 *
	 * This only holds a certain (ground relative) direction and does not perform
	 * cross track correction. Helpful for semi-autonomous modes. Similar to navigateHeading.
	 *
	 * @param[in] bearing Bearing angle [rad]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateBearing(float bearing, const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel);

	/*
	 * Keep the wings level.
	 *
	 * @param[in] heading Heading angle [rad]
	 */
	void navigateLevelFlight(const float heading);

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Set the maximum roll angle output in radians
	 */
	void setRollLimit(float roll_lim_rad) { roll_lim_rad_ = roll_lim_rad; }

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Set roll angle slew rate. Set to zero to deactivate.
	 */
	void setRollSlewRate(float roll_slew_rate) { roll_slew_rate_ = roll_slew_rate; }

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Set control loop dt. The value will be used to apply roll angle setpoint slew rate limiting.
	 */
	void setDt(const float dt) { dt_ = dt; }

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Get the switch distance
	 *
	 * This is the distance at which the system will switch to the next waypoint.
	 * This depends on the period and damping
	 *
	 * @param[in] wp_radius The switching radius the waypoint has set.
	 */
	float switchDistance(float wp_radius) const;

	/*
	 * The path bearing
	 *
	 * @return bearing angle (-pi..pi, in NED frame) [rad]
	 */
	float targetBearing() const { return atan2f(unit_path_tangent_(1), unit_path_tangent_(0)); }

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Returns true if the loiter waypoint has been reached
	 */
	bool reachedLoiterTarget() { return circleMode(); }

	/*
	 * Returns true if following a circle (loiter)
	 */
	bool circleMode() { return path_type_loiter_ && track_proximity_ > EPSILON; }

	/*
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Get roll angle setpoint for fixed wing.
	 *
	 * @return Roll angle (in NED frame)
	 */
	float getRollSetpoint() { return roll_setpoint_; }

private:

	static constexpr float EPSILON = 1.0e-4;
	static constexpr float MIN_RADIUS = 0.5f; // minimum effective radius (avoid singularities) [m]
	static constexpr float PERIOD_SAFETY_FACTOR = 4.0f; // multiplier for period lower bound [s]

	float period_{20.0f}; // nominal (desired) period -- user defined [s]
	float damping_{0.7071f}; // nominal (desired) damping ratio -- user defined
	float p_gain_{0.4442}; // proportional gain (computed from period_ and damping_) [rad/s]
	float time_const_{14.142f}; // time constant (computed from period_ and damping_) [s]
	float adapted_period_{20.0f}; // auto-adapted period (if stability bounds enabled) [s]

	bool en_period_lb_{true}; // enables automatic lower bound constraints on controller period
	bool en_period_ub_{true}; // enables automatic upper bound constraints on controller period (remains disabled if lower bound is disabled)
	bool ramp_in_adapted_period_{true}; // linearly ramps in upper bounded period adaptations from the nominal user setting according to track proximity

	bool en_min_ground_speed_{true}; // the airspeed reference is incremented to sustain a user defined minimum forward ground speed
	bool en_track_keeping_{false}; // the airspeed reference is incremented to return to the track and sustain zero ground velocity until excess wind subsides
	bool en_wind_excess_regulation_{true}; // the airspeed reference is incremented to regulate the excess wind, but not overcome it ...
	// ^disabling this parameter disables all other excess wind handling options, using only the nominal airspeed for reference
	float min_gsp_cmd_{0.0f}; // user defined miminum forward ground speed [m/s]
	float min_gsp_track_keeping_{0.0f}; // minimum forward ground speed demand from track keeping logic [m/s]
	float min_gsp_track_keeping_max_{5.0f}; // maximum, minimum forward ground speed demand from track keeping logic [m/s]
	float min_ground_speed_ref_{0.0f}; // resultant minimum forward ground speed reference considering all active guidance logic [m/s]
	float inv_nte_fraction_{0.5f}; // inverse normalized track error fraction ...
	// ^determines at what fraction of the normalized track error the maximum track keeping forward ground speed demand is reached
	float feas_{1.0f}; // continous representation of bearing feasibility in [0,1] (0=infeasible, 1=feasible)
	float feas_on_track_{1.0f}; // continuous bearing feasibility "on track"
	float airspeed_buffer_{1.5f}; // size of the region above the feasibility boundary (into feasible space) where a continuous transition from feasible to infeasible is imposed [m/s]

	float track_error_bound_{135.0f}; // the current ground speed dependent track error bound [m]
	float track_proximity_{0.0f}; // value in [0,1] indicating proximity to track, 0 = at track error boundary or beyond, 1 = on track

	float airspeed_nom_{15.0f}; // nominal (desired) airspeed reference (generally equivalent to cruise optimized airspeed) [m/s]
	float airspeed_max_{20.0f}; // maximum airspeed reference - the maximum achievable/allowed airspeed reference [m/s]
	float roll_time_const_{0.5f}; // autopilot roll response time constant [s]

	matrix::Vector2f bearing_vec_{matrix::Vector2f{1.0f, 0.0f}}; // bearing unit vector
	float airspeed_ref_{15.0f}; // airspeed reference [m/s]
	matrix::Vector2f air_vel_ref_{matrix::Vector2f{15.0f, 0.0f}}; // air velocity reference vector [m/s]
	float lateral_accel_{0.0f}; // lateral acceleration reference [m/s^2]
	float lateral_accel_ff_{0.0f}; // lateral acceleration demand to maintain path curvature [m/s^2]

	/* ECL_L1_Pos_Controller functionality */
	float dt_{0}; // control loop time [s]
	float roll_lim_rad_{math::radians(30.0f)}; // maximum roll angle [rad]
	float roll_setpoint_{0.0f}; // current roll angle setpoint [rad]
	float roll_slew_rate_{0.0f}; // roll angle setpoint slew rate limit [rad/s]
	bool circle_mode_{false}; // true if following circle
	bool path_type_loiter_{false}; // true if the guidance law is tracking a loiter circle
	matrix::Vector2f unit_path_tangent_{matrix::Vector2f{1.0f, 0.0f}}; // unit path tangent vector
	float signed_track_error_{0.0f}; // signed track error [m]

	/*
	 * Computes the lateral acceleration and airspeed references necessary to track
	 * a path in wind (including excess wind conditions).
	 *
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] unit_path_tangent Unit vector tangent to path at closest point
	 *            in direction of path
	 * @param[in] signed_track_error Signed error to track at closest point (sign
	 *            determined by path normal direction) [m]
	 * @param[in] path_curvature Path curvature at closest point on track [m^-1]
	 * @param[in] in_front_of_wp True if we are in front of the starting point of
	 *            a path segment
	 * @param[in] bearing_vec_to_point Bearing vector to starting point of path
	 *            segment, if relevant
	 */
	void evaluate(const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel,
		      matrix::Vector2f &unit_path_tangent, float signed_track_error,
		      const float path_curvature, bool in_front_of_wp = false,
		      const matrix::Vector2f &bearing_vec_to_point = matrix::Vector2f{0.0f, 0.0f});

	/*
	 * Adapts the controller period considering user defined inputs, current flight
	 * condition, path properties, and stability bounds.
	 *
	 * @param[in] ground_speed Vehicle ground speed [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @param[in] track_error Track error (magnitude) [m]
	 * @param[in] path_curvature Path curvature at closest point on track [m^-1]
	 * @param[in] wind_vel Wind velocity vector in inertial frame [m/s]
	 * @param[in] unit_path_tangent Unit vector tangent to path at closest point
	 *            in direction of path
	 * @param[in] feas_on_track Bearing feasibility on track at the closest point
	 * @return Adapted period [s]
	 */
	float adaptPeriod(const float ground_speed, const float airspeed, const float wind_speed,
			  const float track_error, const float path_curvature, const matrix::Vector2f &wind_vel,
			  const matrix::Vector2f &unit_path_tangent, const float feas_on_track) const;

	/*
	 * Returns normalized (unitless) and constrained track error [0,1].
	 *
	 * @param[in] track_error Track error (magnitude) [m]
	 * @param[in] track_error_bound Track error boundary [m]
	 * @return Normalized track error
	 */
	float normalizedTrackError(const float track_error, const float track_error_bound) const;

	/*
	 * Cacluates an approximation of the wind factor (see [TODO: include citation]).
	 *
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @return Non-dimensional wind factor approximation
	 */
	float windFactor(const float airspeed, const float wind_speed) const;

	/*
	 * Calculates a theoretical upper bound on the user defined period to maintain
	 * track keeping stability.
	 *
	 * @param[in] air_turn_rate The turn rate required to track the current path
	 *            curvature at the current airspeed, in a no-wind condition [rad/s]
	 * @param[in] wind_factor Non-dimensional wind factor (see [TODO: include citation])
	 * @return Period upper bound [s]
	 */
	float periodUB(const float air_turn_rate, const float wind_factor, const float feas_on_track) const;

	/*
	 * Calculates a theoretical lower bound on the user defined period to avoid
	 * limit cycle oscillations considering an acceleration actuation delay (e.g.
	 * roll response delay). Note this lower bound defines *marginal stability,
	 * and a safety factor should be applied in addition to the returned value.
	 *
	 * @param[in] air_turn_rate The turn rate required to track the current path
	 *            curvature at the current airspeed, in a no-wind condition [rad/s]
	 * @param[in] wind_factor Non-dimensional wind factor (see [TODO: include citation])
	 * @return Period lower bound [s]
	 */
	float periodLB(const float air_turn_rate, const float wind_factor, const float feas_on_track) const;

	/*
	 * Computes a continous non-dimensional track proximity [0,1] - 0 when the
	 * vehicle is at the track error boundary, and 1 when on track.
	 *
	 * @param[in] look_ahead_ang The angle at which the bearing vector deviates
	 *            from the unit track error vector [rad]
	 * @return Track proximity
	 */
	float trackProximity(const float look_ahead_ang) const;

	/*
	 * Calculates a ground speed modulated track error bound under which the
	 * look ahead angle is quadratically transitioned from alignment with the
	 * track error vector to that of the path tangent vector.
	 *
	 * @param[in] ground_speed Vehicle ground speed [m/s]
	 * @param[in] time_const Controller time constant [s]
	 * @return Track error boundary [m]
	 */
	float trackErrorBound(const float ground_speed, const float time_const) const;

	/*
	 * Calculates the required controller proportional gain to achieve the desired
	 * system period and damping ratio. NOTE: actual period and damping will vary
	 * when following paths with curvature in wind.
	 *
	 * @param[in] period Desired system period [s]
	 * @param[in] damping Desired system damping ratio
	 * @return Proportional gain [rad/s]
	 */
	float pGain(const float period, const float damping) const;

	/*
	 * Calculates the required controller time constant to achieve the desired
	 * system period and damping ratio. NOTE: actual period and damping will vary
	 * when following paths with curvature in wind.
	 *
	 * @param[in] period Desired system period [s]
	 * @param[in] damping Desired system damping ratio
	 * @return Time constant [s]
	 */
	float timeConst(const float period, const float damping) const;

	/*
	 * Cacluates the look ahead angle as a quadratic function of the normalized
	 * track error.
	 *
	 * @param[in] normalized_track_error Normalized track error (track error / track error boundary)
	 * @return Look ahead angle [rad]
	 */
	float lookAheadAngle(const float normalized_track_error) const;

	/*
	 * Calculates the bearing vector and track proximity transitioning variable
	 * from the look-ahead angle mapping.
	 *
	 * @param[in] unit_path_tangent Unit vector tangent to path at closest point
	 *            in direction of path
	 * @param[in] look_ahead_ang The bearing vector lies at this angle from
	 *            the path normal vector [rad]
	 * @param[in] signed_track_error Signed error to track at closest point (sign
	 *            determined by path normal direction) [m]
	 * @return Unit bearing vector
	 */
	matrix::Vector2f bearingVec(const matrix::Vector2f &unit_path_tangent, const float look_ahead_ang,
				    const float signed_track_error) const;

	/*
	 * Calculates the minimum forward ground speed demand for minimum forward
	 * ground speed maintanence as well as track keeping logic.
	 *
	 * @param[in] normalized_track_error Normalized track error (track error / track error boundary)
	 * @param[in] feas Bearing feasibility
	 * @return Minimum forward ground speed demand [m/s]
	 */
	float minGroundSpeed(const float normalized_track_error, const float feas);

	/*
	 * Determines a reference air velocity *without curvature compensation, but
	 * including "optimal" airspeed reference compensation in excess wind conditions.
	 * Nominal and maximum airspeed member variables must be set before using this method.
	 *
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] bearing_vec Bearing vector
	 * @param[in] wind_cross_bearing 2D cross product of wind velocity and bearing vector [m/s]
	 * @param[in] wind_dot_bearing 2D dot product of wind velocity and bearing vector [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @param[in] min_ground_speed Minimum commanded forward ground speed [m/s]
	 * @return Air velocity vector [m/s]
	 */
	matrix::Vector2f refAirVelocity(const matrix::Vector2f &wind_vel, const matrix::Vector2f &bearing_vec,
					const float wind_cross_bearing, const float wind_dot_bearing, const float wind_speed,
					const float min_ground_speed) const;

	/*
	 * Projection of the air velocity vector onto the bearing line considering
	 * a connected wind triangle.
	 *
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_cross_bearing 2D cross product of wind velocity and bearing vector [m/s]
	 * @return Projection of air velocity vector on bearing vector [m/s]
	 */
	float projectAirspOnBearing(const float airspeed, const float wind_cross_bearing) const;

	/*
	 * Check for binary bearing feasibility.
	 *
	 * @param[in] wind_cross_bearing 2D cross product of wind velocity and bearing vector [m/s]
	 * @param[in] wind_dot_bearing 2D dot product of wind velocity and bearing vector [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @return Binary bearing feasibility: 1 if feasible, 0 if infeasible
	 */
	int bearingIsFeasible(const float wind_cross_bearing, const float wind_dot_bearing, const float airspeed,
			      const float wind_speed) const;

	/*
	 * Air velocity solution for a given wind velocity and bearing vector assuming
	 * a "high speed" (not backwards) solution in the excess wind case.
	 *
	 * @param[in] wind_cross_bearing 2D cross product of wind velocity and bearing vector [m/s]
	 * @param[in] airsp_dot_bearing 2D dot product of air velocity (solution) and bearing vector [m/s]
	 * @param[in] bearing_vec Bearing vector
	 * @return Air velocity vector [m/s]
	 */
	matrix::Vector2f solveWindTriangle(const float wind_cross_bearing, const float airsp_dot_bearing,
					   const matrix::Vector2f &bearing_vec) const;


	/*
	 * Air velocity solution for an infeasible bearing.
	 *
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] bearing_vec Bearing vector
	 * @param[in] wind_speed Wind speed [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @return Air velocity vector [m/s]
	 */
	matrix::Vector2f infeasibleAirVelRef(const matrix::Vector2f &wind_vel, const matrix::Vector2f &bearing_vec,
					     const float wind_speed, const float airspeed) const;


	/*
	 * Cacluates a continuous representation of the bearing feasibility from [0,1].
	 * 0 = infeasible, 1 = fully feasible, partial feasibility in between.
	 *
	 * @param[in] wind_cross_bearing 2D cross product of wind velocity and bearing vector [m/s]
	 * @param[in] wind_dot_bearing 2D dot product of wind velocity and bearing vector [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @return bearing feasibility
	 */
	float bearingFeasibility(float wind_cross_bearing, const float wind_dot_bearing, const float airspeed,
				 const float wind_speed) const;

	/*
	 * Calculates an additional feed-forward lateral acceleration demand considering
	 * the path curvature.
	 *
	 * @param[in] unit_path_tangent Unit vector tangent to path at closest point
	 *            in direction of path
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @param[in] wind_speed Wind speed [m/s]
	 * @param[in] signed_track_error Signed error to track at closest point (sign
	 *             determined by path normal direction) [m]
	 * @param[in] path_curvature Path curvature at closest point on track [m^-1]
	 * @return Feed-forward lateral acceleration command [m/s^2]
	 */
	float lateralAccelFF(const matrix::Vector2f &unit_path_tangent, const matrix::Vector2f &ground_vel,
			     const float wind_dot_upt, const float wind_cross_upt, const float airspeed,
			     const float wind_speed, const float signed_track_error, const float path_curvature) const;

	/*
	 * Calculates a lateral acceleration demand from the heading error.
	 * (does not consider path curvature)
	 *
	 * @param[in] air_vel Vechile air velocity vector [m/s]
	 * @param[in] air_vel_ref Reference air velocity vector [m/s]
	 * @param[in] airspeed Vehicle airspeed [m/s]
	 * @return Lateral acceleration demand [m/s^2]
	 */
	float lateralAccel(const matrix::Vector2f &air_vel, const matrix::Vector2f &air_vel_ref,
			   const float airspeed) const;

	/*
	 * Calculates two-dimensional "cross product" of two vectors.
	 * TODO: move to matrix lib (Vector2 operation)
	 *
	 * @param[in] vec_1 Vector 1
	 * @param[in] vec_2 Vector 2
	 * @return 2D cross product
	 */
	float cross2D(const matrix::Vector2f &vec_1, const matrix::Vector2f &vec_2) const { return vec_1(0) * vec_2(1) - vec_1(1) * vec_2(0); }

	/*******************************************************************************
	 * PX4 POSITION SETPOINT INTERFACE FUNCTIONS
	 */

	/**
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Convert a 2D vector from WGS84 to planar coordinates.
	 *
	 * This converts from latitude and longitude to planar
	 * coordinates with (0,0) being at the position of ref and
	 * returns a vector in meters towards wp.
	 *
	 * @param ref The reference position in WGS84 coordinates
	 * @param wp The point to convert to into the local coordinates, in WGS84 coordinates
	 * @return The vector in meters pointing from the reference position to the coordinates
	 */
	matrix::Vector2f getLocalPlanarVector(const matrix::Vector2d &origin, const matrix::Vector2d &target) const;

	/**
	 * [Copied directly from ECL_L1_Pos_Controller]
	 *
	 * Update roll angle setpoint. This will also apply slew rate limits if set.
	 */
	void updateRollSetpoint();

}; // class NPFG

#endif // NPFG_H_
