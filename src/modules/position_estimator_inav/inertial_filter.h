/**
 *   @file: inertial_filter.h
 *
 *   Copyright (C) 2013-2015 Anton Babushkin. All rights reserved.
 *   @author Anton Babushkin <rk3dov@gmail.com>
 */

#include <cmath>
#include <stdbool.h>
#include <px4_eigen.h>
#include <drivers/drv_hrt.h>

using namespace Eigen;

#ifdef __cplusplus
#ifndef isfinite
#define isfinite std::isfinite
#endif
#endif

class InertialFilter
{
public:
	InertialFilter();
	~InertialFilter() {};

	void inertial_filter_predict(float dt, Vector2f &x, float acc);

	void inertial_filter_correct(float e, float dt, Vector2f &x, int i, float w);
};
