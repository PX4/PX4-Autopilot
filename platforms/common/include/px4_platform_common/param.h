/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file param.h
 *
 * C++ Parameter class
 */

#pragma once

#include "param_macros.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <parameters/px4_parameters.hpp>

/**
 * get the parameter handle from a parameter enum
 */
inline static param_t param_handle(px4::params p)
{
	return (param_t)p;
}


/**
 * PARAMETERS() declares the parameters of a class as a plain struct
 *
 *   PARAMETERS(
 *           MC_ROLLRATE_P,
 *           MC_BAT_SCALE_EN
 *   )
 *
 * adds the member
 *
 *   struct { float MC_ROLLRATE_P; int32_t MC_BAT_SCALE_EN; } _param;
 *
 * and a static list of the parameters' handles. Each field has the type of the parameter's
 * definition, where a boolean is an int32_t. param_get_values() fills the struct from the
 * list, at construction and on every updateParams().
 *
 * Every field is a float or an int32_t, so all members of the struct have one size and one
 * alignment. A static_assert on its size rules out padding, which puts field i at offset
 * 4 * i. That is the address the loader writes.
 *
 * To write a parameter back, assign the field and store it:
 *
 *   _param.MC_ROLLRATE_P = 0.2f;
 *   _param.commit(_param.MC_ROLLRATE_P);
 *
 * A class that derives from another class with parameters uses PARAMETERS_CUSTOM_PARENT()
 * in both. Its own _param hides the parent's, so each class lists every parameter its own
 * code reads, also one the parent lists. Both copies load in the same updateParams().
 */
namespace px4
{

template<bool is_float>
struct param_value {
	using type = int32_t;
};

template<>
struct param_value<true> {
	using type = float;
};

/// The C++ type of a parameter's value, from its definition
template<params p>
using param_value_t = typename param_value<parameters_type[(int)p] == PARAM_TYPE_FLOAT>::type;

#define _PARAMETER_INLINE __attribute__((always_inline))

/**
 * Base of a PARAMETERS() struct, to store, reload or reset one of its fields
 *
 * A field's position in the struct is its position in `handles`, so a call names only the
 * field. The methods that take one field are always inlined, which folds its handle to a
 * constant at the call site.
 */
template<auto &handles>
struct ParameterStruct {
	/// The handle of `field`, or PARAM_INVALID if it is not a field of this struct
	_PARAMETER_INLINE param_t handle(const void *field) const
	{
		const size_t index = (static_cast<const uint8_t *>(field) - reinterpret_cast<const uint8_t *>(this)) / sizeof(float);
		return index < sizeof(handles) / sizeof(handles[0]) ? handles[index] : PARAM_INVALID;
	}

	/// Store a field to the parameter storage (@see param_set())
	_PARAMETER_INLINE bool commit(const float &field) const { return param_set(handle(&field), &field) == 0; }
	_PARAMETER_INLINE bool commit(const int32_t &field) const { return param_set(handle(&field), &field) == 0; }

	/// Store a field to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	_PARAMETER_INLINE bool commit_no_notification(const float &field) const { return param_set_no_notification(handle(&field), &field) == 0; }
	_PARAMETER_INLINE bool commit_no_notification(const int32_t &field) const { return param_set_no_notification(handle(&field), &field) == 0; }

	/// Set a field and store it if the value is new. Returns true if the value changed.
	bool commit_no_notification(float &field, float value) const
	{
		if (fabsf(value - field) > FLT_EPSILON) {
			field = value;
			commit_no_notification(field);
			return true;
		}

		return false;
	}

	bool commit_no_notification(int32_t &field, int32_t value) const
	{
		if (value != field) {
			field = value;
			commit_no_notification(field);
			return true;
		}

		return false;
	}

	/// Reload a field from the parameter storage
	_PARAMETER_INLINE bool update(float &field) const { return param_get(handle(&field), &field) == 0; }
	_PARAMETER_INLINE bool update(int32_t &field) const { return param_get(handle(&field), &field) == 0; }

	/// Reset a field's parameter to its default and reload it
	_PARAMETER_INLINE void reset(float &field) const { param_reset_no_notification(handle(&field)); update(field); }
	_PARAMETER_INLINE void reset(int32_t &field) const { param_reset_no_notification(handle(&field)); update(field); }
};

} // namespace px4

/**
 * Load `count` parameters into consecutive 4-byte values starting at `values`.
 * Implemented in module_params.cpp.
 */
__EXPORT void param_get_values(const param_t *handles, unsigned count, void *values, bool mark_used);

#define _PARAMETER_FIELD(name) px4::param_value_t<px4::params::name> name;

#define _PARAMETER_HANDLE(name) static_cast<param_t>(px4::params::name),

#define _PARAMETER_COUNT (sizeof(_param_list) / sizeof(param_t))

#define _DEFINE_PARAMETER_STRUCT(...) \
	static constexpr param_t _param_list[] = {APPLY_ALL(_PARAMETER_HANDLE, __VA_ARGS__)}; \
	struct ParameterValues : px4::ParameterStruct<_param_list> { \
		ParameterValues() { param_get_values(_param_list, _PARAMETER_COUNT, this, true); } \
		APPLY_ALL(_PARAMETER_FIELD, __VA_ARGS__) \
	} _param; \
	static_assert(__is_standard_layout(ParameterValues) && sizeof(ParameterValues) == _PARAMETER_COUNT * sizeof(float), \
		      "parameter struct is not an array of values");

// As with DEFINE_PARAMETERS(), the update method is final so that a class deriving from one
// with parameters fails to build until both use PARAMETERS_CUSTOM_PARENT().
#define PARAMETERS(...) \
	_DEFINE_PARAMETER_STRUCT(__VA_ARGS__) \
	protected: \
	void updateParamsImpl() final { param_get_values(_param_list, _PARAMETER_COUNT, &_param, false); } \
	private:

#define PARAMETERS_CUSTOM_PARENT(parent_class, ...) \
	_DEFINE_PARAMETER_STRUCT(__VA_ARGS__) \
	protected: \
	void updateParamsImpl() override { \
		parent_class::updateParamsImpl(); \
		param_get_values(_param_list, _PARAMETER_COUNT, &_param, false); \
	} \
	private:


/**
 * A parameter table for a struct that is defined elsewhere
 *
 * A library that must not depend on PX4 keeps its parameters in a struct of its own. A table
 * of PARAMETER_AT() entries names the member each parameter loads into, and
 * param_get_members() loads them all:
 *
 *   static constexpr px4::param_member_s _table[] = {
 *           PARAMETER_AT(EKF2_GYR_NOISE, estimator::parameters, ekf2_gyr_noise),
 *   };
 *
 * An entry builds only if the struct is standard-layout, which offsetof needs, and the member
 * has the type of the parameter's definition.
 */
namespace px4
{

struct param_member_s {
	param_t handle;
	uint16_t offset;
};

template<params p, size_t offset, typename S>
constexpr param_member_s param_member(param_value_t<p> S::*)
{
	static_assert(__is_standard_layout(S), "offsetof needs a standard-layout struct");
	static_assert(offset <= UINT16_MAX, "member offset does not fit the table");
	return {static_cast<param_t>(p), static_cast<uint16_t>(offset)};
}

} // namespace px4

#define PARAMETER_AT(name, type, member) \
	px4::param_member<px4::params::name, offsetof(type, member)>(&type::member)

/**
 * Load `count` parameters into the members of `object` that `table` names.
 * Implemented in module_params.cpp.
 */
__EXPORT void param_get_members(const px4::param_member_s *table, unsigned count, void *object, bool mark_used);

#define _DEFINE_SINGLE_PARAMETER(x) \
	do_not_explicitly_use_this_namespace::PAIR(x);

#define _CALL_UPDATE(x) \
	STRIP(x).update();

// define the parameter update method, which will update all parameters.
// It is marked as 'final', so that wrong usages lead to a compile error (see below)
#define _DEFINE_PARAMETER_UPDATE_METHOD(...) \
	protected: \
	void updateParamsImpl() final { \
		APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) \
	} \
	private:

// Define a list of parameters. This macro also creates code to update parameters.
// If you get a compile error like:
//   error: virtual function ‘virtual void <class>::updateParamsImpl()’
// It means you have a custom inheritance tree (at least one class with params that inherits from another
// class with params) and you need to use DEFINE_PARAMETERS_CUSTOM_PARENT() for **all** classes in
// that tree.
#define DEFINE_PARAMETERS(...) \
	APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD(__VA_ARGS__)


#define _DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, ...) \
	protected: \
	void updateParamsImpl() override { \
		parent_class::updateParamsImpl(); \
		APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) \
	} \
	private:

#define DEFINE_PARAMETERS_CUSTOM_PARENT(parent_class, ...) \
	APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, __VA_ARGS__)



// This namespace never needs to be used directly. Use the DEFINE_PARAMETERS_CUSTOM_PARENT and
// DEFINE_PARAMETERS macros instead (the Param classes don't depend on using the macro, the macro just
// makes sure that update() is automatically called).
namespace do_not_explicitly_use_this_namespace
{

template<typename T, px4::params p>
class Param
{
};

// We use partial template specialization for each param type. This is only supported for classes, not individual methods,
// which is why we have to repeat the whole class
template<px4::params p>
class Param<float, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

	Param()
	{
		param_get_mark_used(handle(), &_val);
	}

	float get() const { return _val; }

	const float &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), &_val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(float val)
	{
		if (fabsf(val - _val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(float val) { _val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), &_val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	float _val;
};

// external version
template<px4::params p>
class Param<float &, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

	Param(float &external_val)
		: _val(external_val)
	{
		param_get_mark_used(handle(), &_val);
	}

	float get() const { return _val; }

	const float &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), &_val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(float val)
	{
		if (fabsf(val - _val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(float val) { _val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), &_val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	float &_val;
};

template<px4::params p>
class Param<int32_t, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	Param()
	{
		param_get_mark_used(handle(), &_val);
	}

	int32_t get() const { return _val; }

	const int32_t &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), &_val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(int32_t val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(int32_t val) { _val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), &_val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	int32_t _val;
};

//external version
template<px4::params p>
class Param<int32_t &, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	Param(int32_t &external_val)
		: _val(external_val)
	{
		param_get_mark_used(handle(), &_val);
	}

	int32_t get() const { return _val; }

	const int32_t &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), &_val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), &_val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(int32_t val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(int32_t val) { _val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), &_val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	int32_t &_val;
};

template<px4::params p>
class Param<bool, p>
{
public:
	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	Param()
	{
		int32_t value_int;

		if (param_get_mark_used(handle(), &value_int) == 0) {
			_val = value_int != 0;
		}
	}

	bool get() const { return _val; }

	const bool &reference() const { return _val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const
	{
		int32_t value_int = (int32_t)_val;
		return param_set(handle(), &value_int) == 0;
	}

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const
	{
		int32_t value_int = (int32_t)_val;
		return param_set_no_notification(handle(), &value_int) == 0;
	}

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(bool val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(bool val) { _val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update()
	{
		int32_t value_int;
		int ret = param_get(handle(), &value_int);

		if (ret == 0) {
			_val = value_int != 0;
			return true;
		}

		return false;
	}

	param_t handle() const { return param_handle(p); }
private:
	bool _val;
};

template <px4::params p>
using ParamFloat = Param<float, p>;

template <px4::params p>
using ParamInt = Param<int32_t, p>;

template <px4::params p>
using ParamExtFloat = Param<float &, p>;

template <px4::params p>
using ParamExtInt = Param<int32_t &, p>;

template <px4::params p>
using ParamBool = Param<bool, p>;

} /* namespace do_not_explicitly_use_this_namespace */


// Raise an appropriate compile error if a Param class is used directly (just to simplify debugging)
template<px4::params p>
class ParamInt
{
	static_assert((int)p &&false, "Do not use this class directly, use the DEFINE_PARAMETERS macro instead");
};
template<px4::params p>
class ParamFloat
{
	static_assert((int)p &&false, "Do not use this class directly, use the DEFINE_PARAMETERS macro instead");
};
