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
 * @file param_macros.h
 *
 * Helper macros used by px4_param.h
 */

#pragma once

#define PX4_PP_CAT(a, b) PX4_PP_CAT_IMPL(a, b)
#define PX4_PP_CAT_IMPL(a, b) a ## b

#define PX4_PP_PROBE() ~, 1
#define PX4_PP_SECOND(a, b, ...) b
#define PX4_PP_IS_PROBE(...) PX4_PP_SECOND(__VA_ARGS__, 0)

#define PX4_PP_IS_ENABLED(value) PX4_PP_IS_PROBE(PX4_PP_CAT(PX4_PP_IS_ENABLED_, value))
#define PX4_PP_IS_ENABLED_1 PX4_PP_PROBE()

#define PX4_PP_IF(condition) PX4_PP_CAT(PX4_PP_IF_, condition)
#define PX4_PP_IF_0(then_clause, else_clause) else_clause
#define PX4_PP_IF_1(then_clause, else_clause) then_clause

#define PX4_PP_OR(left, right) PX4_PP_CAT(PX4_PP_OR_, left)(right)
#define PX4_PP_OR_0(right) right
#define PX4_PP_OR_1(right) 1
#define PX4_PP_OR3(a, b, c) PX4_PP_OR(PX4_PP_OR(a, b), c)

#define PX4_PP_AND(left, right) PX4_PP_CAT(PX4_PP_AND_, left)(right)
#define PX4_PP_AND_0(right) 0
#define PX4_PP_AND_1(right) right
#define PX4_PP_AND2(a, b) PX4_PP_AND(a, b)

// Apply a macro to each argument in a variadic list. This avoids the generated
// APPLY0/APPLY1/... table and keeps macro invocations below MSVC's argument
// limit when used together with DEFINE_PARAMETERS_GROUPED().

#define PX4_PP_EVAL0(...) __VA_ARGS__
#define PX4_PP_EVAL1(...) PX4_PP_EVAL0(PX4_PP_EVAL0(PX4_PP_EVAL0(__VA_ARGS__)))
#define PX4_PP_EVAL2(...) PX4_PP_EVAL1(PX4_PP_EVAL1(PX4_PP_EVAL1(__VA_ARGS__)))
#define PX4_PP_EVAL3(...) PX4_PP_EVAL2(PX4_PP_EVAL2(PX4_PP_EVAL2(__VA_ARGS__)))
#define PX4_PP_EVAL4(...) PX4_PP_EVAL3(PX4_PP_EVAL3(PX4_PP_EVAL3(__VA_ARGS__)))
#define PX4_PP_EVAL(...) PX4_PP_EVAL4(__VA_ARGS__)

#define PX4_PP_MAP_END(...)
#define PX4_PP_MAP_OUT
#define PX4_PP_MAP_GET_END2() 0, PX4_PP_MAP_END
#define PX4_PP_MAP_GET_END1(...) PX4_PP_MAP_GET_END2
#define PX4_PP_MAP_GET_END(...) PX4_PP_MAP_GET_END1
#define PX4_PP_MAP_NEXT0(test, next, ...) next PX4_PP_MAP_OUT
#define PX4_PP_MAP_NEXT1(test, next) PX4_PP_MAP_NEXT0(test, next, 0)
#define PX4_PP_MAP_NEXT(test, next) PX4_PP_MAP_NEXT1(PX4_PP_MAP_GET_END test, next)

#define PX4_PP_MAP0(macro, x, peek, ...) macro(x) PX4_PP_MAP_NEXT(peek, PX4_PP_MAP1)(macro, peek, __VA_ARGS__)
#define PX4_PP_MAP1(macro, x, peek, ...) macro(x) PX4_PP_MAP_NEXT(peek, PX4_PP_MAP0)(macro, peek, __VA_ARGS__)
#define APPLY_ALL(macro, ...) PX4_PP_EVAL(PX4_PP_MAP1(macro, __VA_ARGS__, ()()(), ()()(), ()()(), 0))

// Grouped parameter declarations need a mapper that can emit APPLY_ALL() calls
// for the parameters inside each group. Keep that outer mapper separate so the
// preprocessor does not suppress the inner mapper as a self-reference.
#define PX4_PP_GROUP_EVAL0(...) __VA_ARGS__
#define PX4_PP_GROUP_EVAL1(...) PX4_PP_GROUP_EVAL0(PX4_PP_GROUP_EVAL0(PX4_PP_GROUP_EVAL0(__VA_ARGS__)))
#define PX4_PP_GROUP_EVAL2(...) PX4_PP_GROUP_EVAL1(PX4_PP_GROUP_EVAL1(PX4_PP_GROUP_EVAL1(__VA_ARGS__)))
#define PX4_PP_GROUP_EVAL3(...) PX4_PP_GROUP_EVAL2(PX4_PP_GROUP_EVAL2(PX4_PP_GROUP_EVAL2(__VA_ARGS__)))
#define PX4_PP_GROUP_EVAL4(...) PX4_PP_GROUP_EVAL3(PX4_PP_GROUP_EVAL3(PX4_PP_GROUP_EVAL3(__VA_ARGS__)))
#define PX4_PP_GROUP_EVAL(...) PX4_PP_GROUP_EVAL4(__VA_ARGS__)

#define PX4_PP_GROUP_MAP_END(...)
#define PX4_PP_GROUP_MAP_OUT
#define PX4_PP_GROUP_MAP_GET_END2() 0, PX4_PP_GROUP_MAP_END
#define PX4_PP_GROUP_MAP_GET_END1(...) PX4_PP_GROUP_MAP_GET_END2
#define PX4_PP_GROUP_MAP_GET_END(...) PX4_PP_GROUP_MAP_GET_END1
#define PX4_PP_GROUP_MAP_NEXT0(test, next, ...) next PX4_PP_GROUP_MAP_OUT
#define PX4_PP_GROUP_MAP_NEXT1(test, next) PX4_PP_GROUP_MAP_NEXT0(test, next, 0)
#define PX4_PP_GROUP_MAP_NEXT(test, next) PX4_PP_GROUP_MAP_NEXT1(PX4_PP_GROUP_MAP_GET_END test, next)

#define PX4_PP_GROUP_MAP0(macro, x, peek, ...) macro(x) PX4_PP_GROUP_MAP_NEXT(peek, PX4_PP_GROUP_MAP1)(macro, peek, __VA_ARGS__)
#define PX4_PP_GROUP_MAP1(macro, x, peek, ...) macro(x) PX4_PP_GROUP_MAP_NEXT(peek, PX4_PP_GROUP_MAP0)(macro, peek, __VA_ARGS__)
#define APPLY_ALL_GROUPS(macro, ...) PX4_PP_GROUP_EVAL(PX4_PP_GROUP_MAP1(macro, __VA_ARGS__, ()()(), ()()(), ()()(), 0))


// helper macros to handle macro arguments in the form: (type) name

#define REM(...) __VA_ARGS__
#define EAT(...)

// Retrieve the type
#define TYPEOF(x) DETAIL_TYPEOF(DETAIL_TYPEOF_PROBE x,)
#define DETAIL_TYPEOF(...) DETAIL_TYPEOF_HEAD(__VA_ARGS__)
#define DETAIL_TYPEOF_HEAD(x, ...) REM x
#define DETAIL_TYPEOF_PROBE(...) (__VA_ARGS__),
// Strip off the type, get the name
#define STRIP(x) EAT x
// Show the type without parenthesis
#define PAIR(x) REM x
