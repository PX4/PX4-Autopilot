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
 * How DEFINE_PARAMETERS loads the parameters of a class
 *
 * Problem: if every parameter loaded itself, each one would cost a param_get_mark_used()
 * call in the class constructor and a param_get() call in updateParamsImpl(). Loading
 * them from a table instead saves about 20 kB of flash on px4_fmu-v6x.
 *
 * Approach: the macro declares the parameters as consecutive members, so one shared
 * function can load them all. The macro adds three things to the class:
 *
 * 1. The Param members, in the order listed. Their constructors do not load a value.
 * 2. _param_table_entries, a static array with one entry per member. An entry is the
 *    parameter handle plus the kind of member: a float or int32_t, a bool, or a pointer
 *    to an external value.
 * 3. _param_table, a member declared after the parameters. Its constructor loads them
 *    with one call to param_table_load(), and updateParamsImpl() makes the same call
 *    when parameters change. Members are constructed in declaration order, so by then
 *    every external parameter holds its pointer.
 *
 * param_table_load() gets the address of the first parameter and no other offsets.
 * For each entry it rounds the address up to the alignment of that kind, loads the
 * value there and advances by the size of that kind. That is how the compiler places
 * consecutive members, so the walk lands on each parameter in turn.
 *
 * The C++ standard does not guarantee that placement, so the build checks it.
 * param_table_entry() asserts that each Param type has the size and alignment the
 * loader assumes for its kind. _CHECK_PARAMETER_LAYOUT asserts that the offsetof of
 * every parameter is where the walk arrives. A compiler that pads differently fails
 * the build instead of loading into the wrong member.
 *
 * A Param declared outside these macros is never loaded.
 */

#define _DEFINE_SINGLE_PARAMETER(x) \
	do_not_explicitly_use_this_namespace::PAIR(x);

#define _PARAM_TABLE_ENTRY(x) \
	do_not_explicitly_use_this_namespace::param_table_entry<decltype(STRIP(x))>(),

// The member name of the first parameter in a list
#define _FIRST_PARAMETER(...) _FIRST_PARAMETER_H(__VA_ARGS__, unused)
#define _FIRST_PARAMETER_H(x, ...) STRIP(x)

// The entries go in a static array of the class because APPLY_ALL leaves a trailing comma,
// which only a braced list accepts.
#define _DEFINE_PARAMETER_TABLE(...) \
	static constexpr param_t _param_table_entries[] = {APPLY_ALL(_PARAM_TABLE_ENTRY, __VA_ARGS__)}; \
	do_not_explicitly_use_this_namespace::ParamTable _param_table{_param_table_entries, &_FIRST_PARAMETER(__VA_ARGS__)};

#define _PARAM_TABLE_OFFSET(x) \
	offsetof(_param_table_class, STRIP(x)),

// Fails the build if any parameter is not where the loader steps. It expands inside a
// member function, where the class is complete and decltype(*this) names it. offsetof on
// a class with virtual functions is only conditionally supported. GCC and Clang support
// it but warn.
#define _CHECK_PARAMETER_LAYOUT(...) \
	using _param_table_class = typename do_not_explicitly_use_this_namespace::param_table_class<decltype(*this)>::type; \
	_Pragma("GCC diagnostic push") \
	_Pragma("GCC diagnostic ignored \"-Winvalid-offsetof\"") \
	static_assert(do_not_explicitly_use_this_namespace::param_table_layout_matches(_param_table_entries, \
			{APPLY_ALL(_PARAM_TABLE_OFFSET, __VA_ARGS__)}), "Parameters are not laid out where the table loader steps"); \
	_Pragma("GCC diagnostic pop")

// define the parameter update method, which will update all parameters.
// It is marked as 'final', so that wrong usages lead to a compile error (see below)
#define _DEFINE_PARAMETER_UPDATE_METHOD(...) \
	protected: \
	void updateParamsImpl() final { \
		_CHECK_PARAMETER_LAYOUT(__VA_ARGS__) \
		_param_table.update(_param_table_entries, &_FIRST_PARAMETER(__VA_ARGS__)); \
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
	_DEFINE_PARAMETER_TABLE(__VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD(__VA_ARGS__)


#define _DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, ...) \
	protected: \
	void updateParamsImpl() override { \
		parent_class::updateParamsImpl(); \
		_CHECK_PARAMETER_LAYOUT(__VA_ARGS__) \
		_param_table.update(_param_table_entries, &_FIRST_PARAMETER(__VA_ARGS__)); \
	} \
	private:

#define DEFINE_PARAMETERS_CUSTOM_PARENT(parent_class, ...) \
	APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) \
	_DEFINE_PARAMETER_TABLE(__VA_ARGS__) \
	_DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, __VA_ARGS__)



// This namespace never needs to be used directly. Use the DEFINE_PARAMETERS_CUSTOM_PARENT and
// DEFINE_PARAMETERS macros instead. Only the table those macros add loads a Param.
namespace do_not_explicitly_use_this_namespace
{

// Table entry: parameter handle in the low bits, how its member stores the value in the top two
static_assert(sizeof(param_t) >= 2);
static constexpr unsigned param_table_kind_shift = 14;
static constexpr param_t param_table_handle_mask = (1u << param_table_kind_shift) - 1;
static constexpr param_t param_table_value = 0u << param_table_kind_shift;    // float or int32_t
static constexpr param_t param_table_bool = 1u << param_table_kind_shift;     // bool
static constexpr param_t param_table_external = 2u << param_table_kind_shift; // pointer to the value

constexpr size_t param_table_member_size(param_t kind)
{
	return kind == param_table_external ? sizeof(void *) : kind == param_table_bool ? sizeof(bool) : sizeof(float);
}

constexpr size_t param_table_member_align(param_t kind)
{
	return kind == param_table_external ? alignof(void *) : kind == param_table_bool ? alignof(bool) : alignof(float);
}

// Where a member of `kind` starts after the previous member ends at `end`
constexpr uintptr_t param_table_member_start(uintptr_t end, param_t kind)
{
	return (end + param_table_member_align(kind) - 1) & ~(param_table_member_align(kind) - 1);
}

/**
 * Load `count` parameters into consecutive members starting at `first`, placing each
 * member at the next address its alignment allows, as the compiler lays them out.
 * Implemented in module_params.cpp.
 */
__EXPORT void param_table_load(const param_t *entries, unsigned count, void *first, bool mark_used);

template<typename P>
constexpr param_t param_table_entry()
{
	constexpr param_t kind = P::table_entry & ~param_table_handle_mask;
	static_assert(sizeof(P) == param_table_member_size(kind) && alignof(P) == param_table_member_align(kind),
		      "Param layout must match the table loader");
	return P::table_entry;
}

/**
 * Whether members at `offsets` within their class sit where param_table_load() steps.
 * The class is aligned to at least each member's alignment, so stepping through offsets
 * pads the same as stepping through addresses.
 */
template<size_t N>
constexpr bool param_table_layout_matches(const param_t (&entries)[N], const size_t (&offsets)[N])
{
	uintptr_t member = offsets[0];

	for (size_t i = 0; i < N; i++) {
		const param_t kind = entries[i] & ~param_table_handle_mask;
		member = param_table_member_start(member, kind);

		if (offsets[i] != member) {
			return false;
		}

		member += param_table_member_size(kind);
	}

	return true;
}

template<typename T>
struct param_table_class;

template<typename T>
struct param_table_class<T &> {
	using type = T;
};

/**
 * Declared after the parameters of a class by DEFINE_PARAMETERS, so it loads them
 * once they are all constructed.
 */
class ParamTable
{
public:
	template<size_t N>
	ParamTable(const param_t (&entries)[N], void *first) { param_table_load(entries, N, first, true); }

	template<size_t N>
	void update(const param_t (&entries)[N], void *first) { param_table_load(entries, N, first, false); }
};

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
	static_assert((int)p <= param_table_handle_mask, "parameter handle does not fit a table entry");
	static constexpr param_t table_entry = (param_t)p | param_table_value;

	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

	// loaded by the ParamTable that follows the parameters
	Param() = default;

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
	static_assert((int)p <= param_table_handle_mask, "parameter handle does not fit a table entry");
	static constexpr param_t table_entry = (param_t)p | param_table_external;

	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

	// loaded by the ParamTable that follows the parameters
	Param(float &external_val)
		: _val(&external_val)
	{
	}

	float get() const { return *_val; }

	const float &reference() const { return *_val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), _val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), _val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(float val)
	{
		if (fabsf(val - *_val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(float val) { *_val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), _val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	float *_val; // a pointer rather than a reference, so the table loader can read it
};

template<px4::params p>
class Param<int32_t, p>
{
public:
	static_assert((int)p <= param_table_handle_mask, "parameter handle does not fit a table entry");
	static constexpr param_t table_entry = (param_t)p | param_table_value;

	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	// loaded by the ParamTable that follows the parameters
	Param() = default;

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
	static_assert((int)p <= param_table_handle_mask, "parameter handle does not fit a table entry");
	static constexpr param_t table_entry = (param_t)p | param_table_external;

	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	// loaded by the ParamTable that follows the parameters
	Param(int32_t &external_val)
		: _val(&external_val)
	{
	}

	int32_t get() const { return *_val; }

	const int32_t &reference() const { return *_val; }

	/// Store the parameter value to the parameter storage (@see param_set())
	bool commit() const { return param_set(handle(), _val) == 0; }

	/// Store the parameter value to the parameter storage, w/o notifying the system (@see param_set_no_notification())
	bool commit_no_notification() const { return param_set_no_notification(handle(), _val) == 0; }

	/// Set and commit a new value. Returns true if the value changed.
	bool commit_no_notification(int32_t val)
	{
		if (val != *_val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}

	void set(int32_t val) { *_val = val; }

	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}

	bool update() { return param_get(handle(), _val) == 0; }

	param_t handle() const { return param_handle(p); }
private:
	int32_t *_val; // a pointer rather than a reference, so the table loader can read it
};

template<px4::params p>
class Param<bool, p>
{
public:
	static_assert((int)p <= param_table_handle_mask, "parameter handle does not fit a table entry");
	static constexpr param_t table_entry = (param_t)p | param_table_bool;

	// static type-check
	static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

	// loaded by the ParamTable that follows the parameters
	Param() = default;

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
