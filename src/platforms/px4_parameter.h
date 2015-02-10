/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file px4_parameter.h
 *
 * PX4 Parameter API
 */
#pragma once
#if defined(__PX4_ROS)
/* includes when building for ros */
#include "ros/ros.h"
#include <string>
#else
/* includes when building for NuttX */
#include <systemlib/param/param.h>
#endif

namespace px4
{

/**
 * Parameter base class
 */
template <typename T>
class __EXPORT ParameterBase
{
public:
	ParameterBase(const char *name, T value) :
		_name(name),
		_value(value)
	{}

	virtual ~ParameterBase() {};

	/**
	 * Update the parameter value and return the value
	 */
	virtual T update() = 0;

	/**
	 * Get the current parameter value
	 */
	virtual T get() = 0;

protected:
	const char *_name; /** The parameter name */
	T _value; /**< The current value of the parameter */

};

#if defined(__PX4_ROS)
template <typename T>
class Parameter :
	public ParameterBase<T>
{
public:
	Parameter(const char *name, T value) :
		ParameterBase<T>(name, value)
	{
		if (!ros::param::has(name)) {
			ros::param::set(name, value);
		}

		update();
	}

	~Parameter() {};

	/**
	 * Update the parameter value
	 */
	T update()
	{
		ros::param::get(this->_name, this->_value);
		return get();
	}

	/**
	 * Get the current parameter value
	 */
	T get()
	{
		return this->_value;
	}

};
#else
template <typename T>
class __EXPORT Parameter :
	public ParameterBase<T>
{
public:
	Parameter(const char *name, T value) :
		ParameterBase<T>(name, value),
		_handle(param_find(name))
	{
		update();
	}

	~Parameter() {};

	/**
	 * Update the parameter value
	 */
	T update()
	{
		if (_handle != PARAM_INVALID) {
			param_get(_handle, &(this->_value));
		}

		return get();
	}

	/**
	 * Get the current parameter value
	 */
	T get()
	{
		return this->_value;
	}

protected:
	param_t _handle;

};
#endif

typedef Parameter<float> ParameterFloat;
typedef Parameter<int> ParameterInt;
}
