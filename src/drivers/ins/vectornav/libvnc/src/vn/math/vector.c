#include "vn/math/vector.h"

#include <stdio.h>

vec3d create_v3d(double x, double y, double z)
{
	vec3d v;

    v.c[0] = x;
    v.c[1] = y;
    v.c[2] = z;

    return v;
}

void vn_v3_init_fa(vec3f* v, const float* fa)
{
	size_t i;

	for (i = 0; i < 3; i++)
		v->c[i] = fa[i];
}

vec3f add_v3f_v3f(vec3f lhs, vec3f rhs)
{
	vec3f r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];

	return r;
}

vec3d add_v3d_v3d(vec3d lhs, vec3d rhs)
{
	vec3d r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];

	return r;
}

vec4f add_v4f_v4f(vec4f lhs, vec4f rhs)
{
	vec4f r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];
	r.c[3] = lhs.c[3] + rhs.c[3];

	return r;
}

vec3f sub_v3f_v3f(vec3f lhs, vec3f rhs)
{
	vec3f r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];

	return r;
}

vec3d sub_v3d_v3d(vec3d lhs, vec3d rhs)
{
	vec3d r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];

	return r;
}

vec4f sub_v4f_v4f(vec4f lhs, vec4f rhs)
{
	vec4f r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];
	r.c[3] = lhs.c[3] - rhs.c[3];

	return r;
}

#if defined(_MSC_VER)
	/* Disable warnings regarding using sprintf_s since these
	 * function signatures do not provide us with information
	 * about the length of 'out'. */
	#pragma warning(push)
	#pragma warning(disable:4996)
#endif

void str_vec3f(char* out, vec3f v)
{
	sprintf(out, "(%f; %f; %f)", v.c[0], v.c[1], v.c[2]);
}

void str_vec3d(char* out, vec3d v)
{
	sprintf(out, "(%f; %f; %f)", v.c[0], v.c[1], v.c[2]);
}

void str_vec4f(char* out, vec4f v)
{
	sprintf(out, "(%f; %f; %f; %f)", v.c[0], v.c[1], v.c[2], v.c[3]);
}


#if defined(_MSC_VER)
	#pragma warning(pop)
#endif
