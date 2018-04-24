#include <unit_test.h>
#include "../Utility/ControlMath.hpp"
#include <mathlib/mathlib.h>
#include <cfloat>

class ControlMathTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool testThrAttMapping();
	bool testPrioritizeVector();
};

bool ControlMathTest::run_tests()
{
	ut_run_test(testThrAttMapping);
	ut_run_test(testPrioritizeVector);

	return (_tests_failed == 0);
}

bool ControlMathTest::testThrAttMapping()
{

	/* expected: zero roll, zero pitch, zero yaw, full thr mag
	 * reasone: thrust pointing full upward
	 */
	matrix::Vector3f thr{0.0f, 0.0f, -1.0f};
	float yaw = 0.0f;
	vehicle_attitude_setpoint_s att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(att.roll_body < FLT_EPSILON);
	ut_assert_true(att.pitch_body < FLT_EPSILON);
	ut_assert_true(att.yaw_body < FLT_EPSILON);
	ut_assert_true(att.thrust - 1.0f < FLT_EPSILON);

	/* expected: same as before but with 90 yaw
	 * reason: only yaw changed
	 */
	yaw = M_PI_2_F;
	att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(att.roll_body < FLT_EPSILON);
	ut_assert_true(att.pitch_body < FLT_EPSILON);
	ut_assert_true(att.yaw_body - M_PI_2_F < FLT_EPSILON);
	ut_assert_true(att.thrust - 1.0f < FLT_EPSILON);

	/* expected: same as before but roll 180
	 * reason: thrust points straight down and order Euler
	 * order is: 1. roll, 2. pitch, 3. yaw
	 */
	thr = matrix::Vector3f(0.0f, 0.0f, 1.0f);
	att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(fabsf(att.roll_body) - M_PI_F < FLT_EPSILON);
	ut_assert_true(fabsf(att.pitch_body) < FLT_EPSILON);
	ut_assert_true(att.yaw_body - M_PI_2_F < FLT_EPSILON);
	ut_assert_true(att.thrust - 1.0f < FLT_EPSILON);

	/* TODO: find a good way to test it */


	return true;
}

bool ControlMathTest::testPrioritizeVector()
{
	float max = 5.0f;

	// v0 already at max
	matrix::Vector2f v0(max, 0);
	matrix::Vector2f v1(v0(1), -v0(0));
	matrix::Vector2f v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(fabsf(v_r(0)) - max < FLT_EPSILON && v_r(0) > 0.0f);
	ut_assert_true(fabsf(v_r(1) - 0.0f) < FLT_EPSILON);

	// v1 exceeds max but v0 is zero
	v0.zero();
	v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(fabsf(v_r(1)) - max < FLT_EPSILON && v_r(1) < 0.0f);
	ut_assert_true(fabsf(v_r(0) - 0.0f) < FLT_EPSILON);

	// v0 and v1 are below max
	v0 = matrix::Vector2f(0.5f, 0.5f);
	v1 = matrix::Vector2f(v0(1), -v0(0));
	v_r = ControlMath::constrainXY(v0, v1, max);
	float diff = matrix::Vector2f(v_r - (v0 + v1)).length();
	ut_assert_true(diff < FLT_EPSILON);

	// v0 and v1 exceed max and are perpendicular
	v0 = matrix::Vector2f(4.0f, 0.0f);
	v1 = matrix::Vector2f(0.0f, -4.0f);
	v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(v_r(0) - v0(0) < FLT_EPSILON && v_r(0) > 0.0f);
	float remaining = sqrtf(max * max - (v0(0) * v0(0)));
	ut_assert_true(fabsf(v_r(1)) - remaining  < FLT_EPSILON && v_r(1) < FLT_EPSILON);

	//TODO: add more tests with vectors not perpendicular

	return true;

}

ut_declare_test_c(test_controlmath, ControlMathTest)
