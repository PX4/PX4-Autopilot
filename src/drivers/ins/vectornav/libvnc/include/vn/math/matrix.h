#ifndef VN_MATRIX_H_INCLUDED
#define VN_MATRIX_H_INCLUDED

#include "vn/util/compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Represents a 3x3 matrix with an underlying data type of <c>float</c>. */
typedef union
{
	float e[3*3];	/**< The matrix's elements in column-major ordering. */

	/* Check if the compiler supports anonymous unions. */
	#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) &&  defined(__GNUC__)

	struct
	{
		float e00;	/**< Element [0,0]. */
		float e10;	/**< Element [1,0]. */
		float e20;	/**< Element [2,0]. */
		float e01;	/**< Element [0,1]. */
		float e11;	/**< Element [1,1]. */
		float e21;	/**< Element [2,1]. */
		float e02;	/**< Element [0,2]. */
		float e12;	/**< Element [1,2]. */
		float e22;	/**< Element [2,2]. */
	};

	#endif

} mat3f;

/** \brief Represents a quaternion reading with underlying data type of <c>float</c>. */
typedef union
{
	float c[4];		/**< Indexable. */

	/* Check if the compiler supports anonymous unions. */
	#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) &&  defined(__GNUC__)

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

} quatf;

/** \brief Initializes a 3x3 float matrix from an float array with matrix
  * elements in column-major ordering.
  *
  * \param[out] m 3x3 float matrix to initialize
  * \param[in] fa float array containing a 3x3 matrix in column-major order */
void vn_m3_init_fa(mat3f* m, const float* fa);

/** \brief Converts a mat3f to a string.
*
* \param[out] out The char buffer to output the result to.
* \param[in] m The mat3f to convert.
*/
void strFromMat3f(char* out, mat3f m);

/** \brief Negates a mat3f.
* \param[in] m Matrix to negate.
* \return Negated matrix. */
mat3f vnm_negative_mat3f(mat3f m);

#ifdef __cplusplus
}
#endif

#endif

