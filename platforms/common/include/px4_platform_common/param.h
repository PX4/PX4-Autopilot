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
 * To write a parameter back, assign the field and store it by naming the parameter:
 *
 *   _param.MC_ROLLRATE_P = 0.2f;
 *   _param.commit<px4::params::MC_ROLLRATE_P>();
 *
 * Naming a parameter that the struct does not hold fails the build. ParameterAccess lists
 * the other methods.
 *
 * A class that derives from another class with parameters uses PARAMETERS_CUSTOM_PARENT()
 * in both. Its own _param hides the parent's, so it reaches a parent's parameter through
 * Parent::_param.
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

/**
 * Stores, reloads and resets single parameters of a class that holds their values
 *
 * Every method names the parameter, so a call cannot pair a handle with the value of another
 * parameter. `Values` provides value<p>(), a pointer to the value of parameter p, which
 * fails the build for a parameter it does not hold.
 */
template<typename Values>
class ParameterAccess
{
public:
	/// Store the value to the parameter storage (@see param_set())
	template<params p>
	bool commit() const { return param_set(param_handle(p), values().template value<p>()) == 0; }

	/// Store the value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	template<params p>
	bool commit_no_notification() const
	{
		return param_set_no_notification(param_handle(p), values().template value<p>()) == 0;
	}

	/// Set the value and store it w/o notification if it is new. Returns true if the value changed.
	template<params p>
	bool commit_no_notification(param_value_t<p> new_value)
	{
		param_value_t<p> *value = values().template value<p>();

		if (differs(*value, new_value)) {
			*value = new_value;
			commit_no_notification<p>();
			return true;
		}

		return false;
	}

	/// Reload the value from the parameter storage
	template<params p>
	bool update() { return param_get(param_handle(p), values().template value<p>()) == 0; }

	/// Reset the parameter to its default and reload the value
	template<params p>
	void reset()
	{
		param_reset_no_notification(param_handle(p));
		update<p>();
	}

private:
	const Values &values() const { return static_cast<const Values &>(*this); }
	Values &values() { return static_cast<Values &>(*this); }

	static bool differs(float a, float b) { return fabsf(a - b) > FLT_EPSILON; }
	static bool differs(int32_t a, int32_t b) { return a != b; }
};

/**
 * Base of the struct that PARAMETERS() declares. A parameter's position in `handles` is the
 * position of its field in the struct.
 */
template<auto &handles>
class ParameterStruct : public ParameterAccess<ParameterStruct<handles>>
{
	friend ParameterAccess<ParameterStruct<handles>>;

	static constexpr int index(param_t handle)
	{
		for (size_t i = 0; i < sizeof(handles) / sizeof(handles[0]); i++) {
			if (handles[i] == handle) {
				return i;
			}
		}

		return -1;
	}

	template<params p>
	const param_value_t<p> *value() const
	{
		constexpr int field = index(static_cast<param_t>(p));
		static_assert(field >= 0, "not a parameter of this struct");
		return static_cast<const param_value_t<p> *>(static_cast<const void *>(this)) + field;
	}

	template<params p>
	param_value_t<p> *value()
	{
		return const_cast<param_value_t<p> *>(static_cast<const ParameterStruct *>(this)->template value<p>());
	}
};

} // namespace px4

/// Load one parameter into `value`. Implemented in module_params.cpp.
__EXPORT void param_get_value(param_t handle, void *value, bool mark_used);

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
 * of PARAMETER_AT() entries names the member each parameter loads into. A ParameterMembers
 * points at the struct, loads it, and stores single parameters as _param does:
 *
 *   static constexpr px4::ParameterMember<estimator::parameters> _table[] = {
 *           PARAMETER_AT(EKF2_GYR_NOISE, estimator::parameters, ekf2_gyr_noise),
 *   };
 *
 *   px4::ParameterMembers<_table> _params{&estimator_parameters};
 *
 *   _params->ekf2_gyr_noise = 0.1f;
 *   _params.commit<px4::params::EKF2_GYR_NOISE>();
 *
 * An entry builds only if the struct is standard-layout, which offsetof needs, and the member
 * has the type of the parameter's definition.
 */
namespace px4
{

/// The member of S that a parameter loads into
template<typename S>
struct ParameterMember {
	param_t handle;
	uint16_t offset;
};

template<params p, size_t offset, typename S>
constexpr ParameterMember<S> parameter_member(param_value_t<p> S::*)
{
	static_assert(__is_standard_layout(S), "offsetof needs a standard-layout struct");
	static_assert(offset <= UINT16_MAX, "member offset does not fit the table");
	return {static_cast<param_t>(p), static_cast<uint16_t>(offset)};
}

/// The struct that a table of ParameterMember entries is for
template<typename S, size_t N>
S parameter_member_struct(const ParameterMember<S> (&)[N]);

/**
 * Points at a struct and loads the parameters of `table` into its members, at construction
 * and on update()
 */
template<auto &table>
class ParameterMembers : public ParameterAccess<ParameterMembers<table>>
{
public:
	using Struct = decltype(parameter_member_struct(table));

	explicit ParameterMembers(Struct *object) : _object(object) { load(true); }

	Struct *operator->() const { return _object; }

	using ParameterAccess<ParameterMembers<table>>::update;

	/// Reload every parameter of the table
	void update() { load(false); }

private:
	friend ParameterAccess<ParameterMembers<table>>;

	void load(bool mark_used) const
	{
		for (const ParameterMember<Struct> &member : table) {
			param_get_value(member.handle, reinterpret_cast<uint8_t *>(_object) + member.offset, mark_used);
		}
	}

	static constexpr int offset(param_t handle)
	{
		for (const ParameterMember<Struct> &member : table) {
			if (member.handle == handle) {
				return member.offset;
			}
		}

		return -1;
	}

	template<params p>
	param_value_t<p> *value() const
	{
		constexpr int member = offset(static_cast<param_t>(p));
		static_assert(member >= 0, "not a parameter of this table");
		return static_cast<param_value_t<p> *>(static_cast<void *>(reinterpret_cast<uint8_t *>(_object) + member));
	}

	Struct *_object;
};

} // namespace px4

#define PARAMETER_AT(name, type, member) \
	px4::parameter_member<px4::params::name, offsetof(type, member)>(&type::member)

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
