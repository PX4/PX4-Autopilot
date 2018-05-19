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
};

bool ControlMathTest::run_tests()
{
	ut_run_test(testThrAttMapping);

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

ut_declare_test_c(test_controlmath, ControlMathTest)
