#ifndef MATHLIB_H
#define MATHLIB_H
#ifdef POSIX_SHARED
#include <Eigen/Dense>
#include <algorithm>
#define M_PI_F 3.14159265358979323846f

namespace math {
	using namespace Eigen;
	using namespace std;

	float constrain(float &val, float min, float max);
	float radians(float degrees);
	float degrees(float radians);
}
#else
#include <mathlib/mathlib.h>
#endif //POSIX_SHARED
#endif //MATHLIB_H