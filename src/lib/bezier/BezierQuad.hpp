/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

/**
 * @file BezierQuad.hpp
 *
 * Quadratic bezier lib
 *
 * A quadratic bezier function/spline is completely defined by three 3D points in space and a time scaling factor.
 * pt0 and pt1 define the start and end points of the spline. ctrl point is a point in space that effects the curvature
 * of the spline. The time scaling factor (= duration) defines the time it takes to travel along the spline from pt0 to
 * pt1.
 * A bezier spline is a continuous function from which position, velocity and acceleration can be extracted. For a given spline,
 * acceleration stays constant.
 */


#pragma once

#include <matrix/math.hpp>

namespace bezier
{
template<typename Tp>
class BezierQuad
{
public:

	using Vector3_t = matrix::Vector<Tp, 3>;

	/**
	 * Empty constructor
	 */
	BezierQuad() :
		_pt0(Vector3_t()), _ctrl(Vector3_t()), _pt1(Vector3_t()), _duration(1.0f) {}

	/**
	 * Constructor from array
	 */
	BezierQuad(const Tp pt0[3], const Tp ctrl[3], const Tp pt1[3], Tp duration = 1.0f) :
		_pt0(Vector3_t(pt0)), _ctrl(Vector3_t(ctrl)), _pt1(Vector3_t(pt1)), _duration(duration) {}

	/**
	 * Constructor from vector
	 */
	BezierQuad(const Vector3_t &pt0, const Vector3_t &ctrl, const Vector3_t &pt1,
		   Tp duration = 1.0f):
		_pt0(pt0), _ctrl(ctrl), _pt1(pt1), _duration(duration) {}


	/*
	 * Get bezier points
	 */
	void getBezier(Vector3_t &pt0, Vector3_t &ctrl, Vector3_t &pt1);

	/*
	 * Return pt0
	 */
	Vector3_t getPt0() {return _pt0;}

	/*
	 * Return ctrl
	 */
	Vector3_t getCtrl() {return _ctrl;}

	/*
	 * Return pt1
	 */
	Vector3_t getPt1() {return _pt1;}

	/**
	 * Set new bezier points and duration
	 */
	void setBezier(const Vector3_t &pt0, const Vector3_t &ctrl, const Vector3_t &pt1,
		       Tp duration = (Tp)1);

	/*
	 * Set duration
	 *
	 * @param time is the total time it takes to travel along the bezier spline.
	 */
	void setDuration(const Tp time) {_duration = time;}

	/**
	 * Return point on bezier point corresponding to time t
	 *
	 * @param t is a time in seconds in between [0, duration]
	 * @return a point on bezier
	 */
	Vector3_t getPoint(const Tp t);

	/*
	 * Distance to closest point given a position
	 *
	 * @param pose is a position in 3D space from which distance to bezier is computed.
	 * @return distance to closest point on bezier
	 */
	Tp getDistToClosestPoint(const Vector3_t &pose);

	/*
	 * Return velocity on bezier corresponding to time t
	 *
	 * @param t is a time in seconds in between [0, duration]
	 * @return velocity vector at time t
	 */
	Vector3_t getVelocity(const Tp t);

	/*
	 * Return acceleration on bezier corresponding to time t
	 *
	 * @return constant acceleration of bezier
	 */
	Vector3_t getAcceleration();

	/*
	 * Get all states on bezier corresponding to time t
	 */
	void getStates(Vector3_t &point, Vector3_t &vel, Vector3_t &acc, const Tp t);

	/*
	 * Get states on bezier which are closest to pose in space
	 *
	 * @param point is a posiiton on the spline that is closest to a given pose
	 * @param vel is the velocity at that given point
	 * @param acc is the acceleration for that spline
	 * @param pose represent a position in space from which closest point is computed
	 */
	void getStatesClosest(Vector3_t &point, Vector3_t &vel, Vector3_t &acc,
			      const Vector3_t pose);

	/*
	 * Compute bezier from velocity at bezier end points and ctrl point
	 *
	 * The bezier end points are fully defined by a given control point ctrl, the duration and
	 * the desired velocity vectors at the end points.
	 */
	void setBezFromVel(const Vector3_t &ctrl, const Vector3_t &vel0, const Vector3_t &vel1,
			   const Tp duration = (Tp)1);

	/*
	 * Return the arc length of a bezier spline
	 *
	 * The arc length is computed with simpsons integration.
	 * @param resolution in meters.
	 */
	Tp getArcLength(const Tp resolution);

private:

	Vector3_t _pt0; /**< Bezier starting point */
	Vector3_t _ctrl; /**< Bezier control point */
	Vector3_t _pt1; /**< bezier end point */
	Tp _duration = (Tp)1; /**< Total time to travle along spline */

	Tp _cached_arc_length = (Tp)0; /**< The saved arc length of the spline */
	Tp _cached_resolution = (Tp)(-1); /**< The resolution used to compute the arc length.
									Negative number means that cache is not up to date. */

	/*
	 * Golden section search
	 */
	Tp _goldenSectionSearch(const Vector3_t &pose);

	/*
	 * Get squared distance from 3D pose in space and a point on bezier.
	 *
	 * @param t is the time in between [0, duration] that defines a point on the bezier.
	 * @param pose is a 3D pose in space.
	 */
	Tp _getDistanceSquared(const Tp t, const Vector3_t &pose);


};

using BezierQuad_f = BezierQuad<float>;
using BezierQuad_d = BezierQuad<double>;
}

// include implementation
#include "BezierQuad.cpp"
