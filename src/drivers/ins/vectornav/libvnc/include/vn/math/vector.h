#ifndef VN_VECTOR_H_INCLUDED
#define VN_VECTOR_H_INCLUDED

#include "vn/util/compiler.h"

/** \brief Various vector types and operations. */

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Represents a 3 component vector with an underlying data type of
*  <c>float</c>. */
typedef union
{
	float c[3];		/**< Indexable. */

	/* Check if the compiler supports anonymous unions. */
	#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) && defined(__GNUC__)

	struct
	{
		float x;	/**< X component. */
		float y;	/**< Y component. */
		float z;	/**< Z component. */
	};

	struct
	{
		float c0;	/**< Component 0. */
		float c1;	/**< Component 1. */
		float c2;	/**< Component 2. */
	};

	#endif
	
} vec3f;

/** \brief Represents a 3 component vector with an underlying data type of
*  <c>double</c>. */
typedef union
{
	double c[3];	/**< Indexable. */

	/* Check if the compiler supports anonymous unions. */
	#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) && defined(__GNUC__)

	struct
	{
		double x;	/**< The x component. */
		double y;	/**< The y component. */
		double z;	/**< The z component. */
	};

	struct
	{
		double c0;	/**< Component 0. */
		double c1;	/**< Component 1. */
		double c2;	/**< Component 2. */
	};

	#endif

} vec3d;

/** \brief Represents a 4 component vector with an underlying data type of
*  <c>float</c>. */
typedef union
{
	float c[4];		/**< Indexable. */

	/* Check if the compiler supports anonymous unions. */
	#if (defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L)) && defined(__GNUC__)
	
	struct
	{
		float x;	/**< The x component. */
		float y;	/**< The y component. */
		float z;	/**< The z component. */
		float w;	/**< The w component. */
	};

	struct
	{
		float c0;	/**< Component 0. */
		float c1;	/**< Component 1. */
		float c2;	/**< Component 2. */
		float c3;	/**< Component 2. */
	};

	#endif

} vec4f;

/** \brief Initializes a 3-dimensional float vector from an float array.
  *
  * \param[out] v 3-dimensional float vector to initialize
  * \param[in] fa float array a 3-componet vector */
void vn_v3_init_fa(vec3f* v, const float* fa);

/** Creates a vec3d initialized with provided values.
 * \param[in] x x-component.
 * \param[in] y y-component.
 * \param[in] z z-component.
 * \return The initialized vec3d. */
vec3d create_v3d(double x, double y, double z);

/** \brief Adds two vec3f together.
*
* \param[in] lhs The lhs vec3f.
* \param[in] rhs The rhs vec3f.
* \return The resulting vec3f from adding lhs and rhs together. */
vec3f add_v3f_v3f(vec3f lhs, vec3f rhs);

/** \brief Adds two vec3d together.
*
* \param[in] lhs The lhs vec3d.
* \param[in] rhs The rhs vec3d.
* \return The resulting vec3d from adding lhs and rhs together. */
vec3d add_v3d_v3d(vec3d lhs, vec3d rhs);

/** \brief Adds two vec4f together.
*
* \param[in] lhs The lhs vec4f.
* \param[in] rhs The rhs vec4f.
* \return The resulting vec4f from adding lhs and rhs together. */
vec4f add_v4f_v4f(vec4f lhs, vec4f rhs);

/** \brief Subtracts a vec3f from another vec3f.
*
* \param[in] lhs The lhs vec3f.
* \param[in] rhs The rhs vec3f.
* \return The resulting vec3f from subtracting rhs from lhs. */
vec3f sub_v3f_v3f(vec3f lhs, vec3f rhs);

/** \brief Subtracts a vec3d from another vec3d.
*
* \param[in] lhs The lhs vec3d.
* \param[in] rhs The rhs vec3d.
* \return The resulting vec3d from subtracting rhs from lhs. */
vec3d sub_v3d_v3d(vec3d lhs, vec3d rhs);

/** \brief Subtracts a vec4f from another vec4f.
*
* \param[in] lhs The lhs vec4f.
* \param[in] rhs The rhs vec4f.
* \return The resulting vec4f from subtracting rhs from lhs. */
vec4f sub_v4f_v4f(vec4f lhs, vec4f rhs);

/** \brief Converts a vec3f to a string.
 *
 * \param[out] out The char buffer to output the result to.
 * \param[in] v The vec3f to convert. */
void str_vec3f(char* out, vec3f v);

/** \brief Converts a vec3d to a string.
*
* \param[out] out The char buffer to output the result to.
* \param[in] v The vec3d to convert. */
void str_vec3d(char* out, vec3d v);

/** \brief Converts a vec4f to a string.
*
* \param[out] out The char buffer to output the result to.
* \param[in] v The vec4f to convert. */
void str_vec4f(char* out, vec4f v);

#ifdef __cplusplus
}
#endif

#endif
