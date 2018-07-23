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
 * @file BerzierQuad.hpp
 *
 * Quadratic bezier lib
 */


#pragma once

#include <px4_defines.h>
#include <stdlib.h>

#include <matrix/math.hpp>

namespace bezier
{
template<typename Tp>
class __EXPORT BezierQuad
{
public:

	using Data = matrix::Vector<Tp, 3>;

	/**
	 * empty constructor
	 */
	BezierQuad(void) :
		_pt0(Data()), _ctrl(Data()), _pt1(Data()), _duration(1.0f) {}

	/**
	 * constructor from array
	 */
	BezierQuad(const Tp pt0[3], const Tp ctrl[3], const Tp pt1[3], Tp duration = 1.0f) :
		_pt0(Data(pt0)), _ctrl(Data(ctrl)), _pt1(Data(pt1)), _duration(duration) {}

	/**
	 * constructor from vector
	 */
	BezierQuad(const Data &pt0, const Data &ctrl, const Data &pt1,
		   Tp duration = 1.0f):
		_pt0(pt0), _ctrl(ctrl), _pt1(pt1), _duration(duration) {}


	/*
	 * return bezier points
	 */
	void getBezier(Data &pt0, Data &ctrl, Data &pt1);

	/*
	 * get pt0
	 */
	Data getPt0() {return _pt0;}

	/*
	 * get ctrl
	 */
	Data getCtrl() {return _ctrl;}

	/*
	 * get pt1
	 */
	Data getPt1() {return _pt1;}

	/**
	 * set new bezier points
	 */
	void setBezier(const Data &pt0, const Data &ctrl, const Data &pt1,
		       Tp duration = 1.0f);

	/*
	* set duration
	*/
	void setDuration(const Tp time) {_duration = time;}

	/**
	 * get point on bezier point corresponding to t
	 */
	Data getPoint(const Tp t);

	/*
	 * Distance to closest point given a position
	 */
	Tp getDistToClosestPoint(const Data &pose);

	/*
	 * get velocity on bezier corresponding to t
	 */
	Data getVelocity(const Tp t);

	/*
	 * get acceleration on bezier corresponding to t
	 */
	Data getAcceleration();

	/*
	 * get states on bezier corresponding to t
	 */
	void getStates(Data &point, Data &vel, Data &acc, const Tp t);

	/*
	 * get states on bezier which are closest to pose
	 */
	void getStatesClosest(Data &point, Data &vel, Data &acc,
			      const Data pose);

	/*
	 * compute bezier from velocity at bezier end points and ctrl point
	 */
	void setBezFromVel(const Data &ctrl, const Data &vel0, const Data &vel1,
			   const Tp duration = 1.0f);

	/*
	 * simpsons inegrattion applied to velocity
	 */
	Tp getArcLength(const Tp resolution);

private:

	/* control points */
	Data _pt0;
	Data _ctrl;
	Data _pt1;
	Tp _duration;

	/* cache */
	Tp _cached_arc_length;
	Tp _cached_resolution = (Tp)(-1); // negative number means that cache is not up to date

	/*
	 * Helper functions
	 */

	/* golden section search */
	Tp _goldenSectionSearch(const Data &pose);

	/*
	 * get distance to point on bezier
	 */
	Tp _getDistanceSquared(const Tp t, const Data &pose);


};

using BezierQuadf = BezierQuad<float>;
using BezierQuadd = BezierQuad<double>;
}

// include implementation
#include "BezierQuad.cpp"
