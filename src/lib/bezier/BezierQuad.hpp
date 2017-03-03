/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file BerzierQuad.hpp
 *
 * Quadratic bezier lib
 */


#pragma once

#include <px4_defines.h>
#include <stdlib.h>

//#include <mathlib/math/Vector.hpp>
#include <matrix/math.hpp>


namespace bezier
{


class __EXPORT BezierQuad
{
public:

	/**
	 * empty constructor
	 */
	BezierQuad(void) :
		_pt0(matrix::Vector3f()), _ctrl(matrix::Vector3f()), _pt1(matrix::Vector3f()), _duration(1.0f) {}


	/**
	 * constructor from array
	 */
	BezierQuad(const float pt0[3], const float ctrl[3], const float pt2[3], float duration = 1.0f) :
		_pt0(matrix::Vector3f(pt0)), _ctrl(matrix::Vector3f(pt0)), _pt1(matrix::Vector3f(pt0)), _duration(duration) {}

	/**
	 * constructor from vector
	 */
	BezierQuad(const matrix::Vector3f &pt0, const matrix::Vector3f &ctrl, const matrix::Vector3f &pt1,
		   float duration = 1.0f):
		_pt0(pt0), _ctrl(ctrl), _pt1(pt1), _duration(1.0f) {}


	/*
	 * return bezier points
	 */
	void getBezier(matrix::Vector3f &pt0, matrix::Vector3f &ctrl, matrix::Vector3f &pt1);

	/*
	 * get pt0
	 */
	matrix::Vector3f getPt0() {return _pt0;}

	/*
	 * get ctrl
	 */
	matrix::Vector3f getCtrl() {return _ctrl;}

	/*
	 * get pt1
	 */
	matrix::Vector3f getPt1() {return _pt1;}

	/**
	 * set new bezier points
	 */
	void setBezier(const matrix::Vector3f &pt0, const matrix::Vector3f &ctrl, const matrix::Vector3f &pt1,
		       float duration = 1.0f);

	/*
	* set duration
	*/
	void setDuration(const float time) {_duration = time;}

	/**
	 * get point on bezier point corresponding to t
	 */
	matrix::Vector3f getPoint(const float t);

	/*
	 * Distance to closest point given a position
	 */
	float getDistToClosestPoint(const matrix::Vector3f &pose);

	/*
	 * get velocity on bezier corresponding to t
	 */
	matrix::Vector3f getVelocity(const float t);

	/*
	 * get acceleration on bezier corresponding to t
	 */
	matrix::Vector3f getAcceleration();

	/*
	 * get states on bezier corresponding to t
	 */
	void getStates(matrix::Vector3f &point, matrix::Vector3f &vel, matrix::Vector3f &acc, const float t);

	/*
	 * get states on bezier which are closest to pose
	 */
	void getStatesClosest(matrix::Vector3f &point, matrix::Vector3f &vel, matrix::Vector3f &acc,
			      const matrix::Vector3f pose);

	/*
	 * compute bezier from velocity at bezier end points and ctrl point
	 */
	void setBezFromVel(const matrix::Vector3f &ctrl, const matrix::Vector3f &vel0, const matrix::Vector3f &vel1,
			   const float duration = 1.0f);

	/*
	 * simpsons inegrattion applied to velocity
	 */
	float getArcLength(const float resolution);

private:

	/* control points */
	matrix::Vector3f _pt0;
	matrix::Vector3f _ctrl;
	matrix::Vector3f _pt1;
	float _duration;


	/*
	 * Helper functions
	 */

	/* golden section search */
	float _goldenSectionSearch(const matrix::Vector3f &pose);

	/*
	 * get distance to point on bezier
	 */
	float _getDistanceSquared(const float t, const matrix::Vector3f &pose);


};
}

