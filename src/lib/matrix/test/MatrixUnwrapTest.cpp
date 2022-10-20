#include <matrix/math.hpp>
#include <gtest/gtest.h>

using namespace matrix;

TEST(MatrixUnwrapTest, UnwrapFloats)
{
	const float M_TWO_PI_F = float(M_PI_F * 2);

	float unwrapped_angles[6] = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25};
	float wrapped_angles[6] = {0.0, 0.25, 0.5, -0.25, 0.0, 0.25};

	for (int i = 0; i < 6; i++) {
		unwrapped_angles[i] *= M_TWO_PI_F;
		wrapped_angles[i] *= M_TWO_PI_F;
	}

	// positive unwrapping
	float last_angle = wrapped_angles[0];

	for (int i = 1; i < 6; i++) {
		last_angle = unwrap_pi(last_angle, wrapped_angles[i]);
		EXPECT_FLOAT_EQ(last_angle, unwrapped_angles[i]);
	}

	// negative unwrapping
	last_angle = -wrapped_angles[0];

	for (int i = 1; i < 6; i++) {
		last_angle = unwrap_pi(last_angle, -wrapped_angles[i]);
		EXPECT_FLOAT_EQ(last_angle, -unwrapped_angles[i]);
	}
}

TEST(MatrixUnwrapTest, UnwrapDoubles)
{
	const double M_TWO_PI = M_PI_PRECISE * 2;

	double unwrapped_angles[6] = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25};
	double wrapped_angles[6] = {0.0, 0.25, 0.5, -0.25, 0.0, 0.25};

	for (int i = 0; i < 6; i++) {
		unwrapped_angles[i] *= M_TWO_PI;
		wrapped_angles[i] *= M_TWO_PI;
	}

	// positive unwrapping
	double last_angle = wrapped_angles[0];

	for (int i = 1; i < 6; i++) {
		last_angle = unwrap_pi(last_angle, wrapped_angles[i]);
		EXPECT_DOUBLE_EQ(last_angle, unwrapped_angles[i]);
	}

	// negative unwrapping
	last_angle = -wrapped_angles[0];

	for (int i = 1; i < 6; i++) {
		last_angle = unwrap_pi(last_angle, -wrapped_angles[i]);
		EXPECT_DOUBLE_EQ(last_angle, -unwrapped_angles[i]);
	}
}
