#include <unit_test.h>
#include <lib/FlightTasks/tasks/Utility/ManualSmoothingZ.hpp>
#include <float.h>

class SmoothZTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool brakeUpward();
	bool brakeDownward();
	bool accelerateUpwardFromBrake();
	bool accelerateDownwardFromBrake();

};

bool SmoothZTest::run_tests()
{
	ut_run_test(brakeUpward);
	ut_run_test(brakeDownward);
	ut_run_test(accelerateUpwardFromBrake);
	ut_run_test(accelerateDownwardFromBrake);

	return (_tests_failed == 0);
}

bool SmoothZTest::brakeUpward()
{
	/* Downward flight and want to stop */
	float stick_current = 0.0f; // sticks are at zero position
	float vel_sp_current = 0.0f; // desired velocity is at 0m/s
	float vel_sp_previous = 5.0f; // the demanded previous setpoint was 5m/s downwards
	float vel = vel_sp_previous; // assume that current velocity is equal to previous vel setpoint
	float acc_max_up = 5.0f;
	float acc_max_down = 2.0f;

	ManualSmoothingZ smooth(nullptr, vel, stick_current);

	/* overwrite parameters since they might change depending on configuration */
	smooth.overwriteAccelerationDown(acc_max_down); // downward max acceleration of 2m/ss
	smooth.overwriteAccelerationUp(acc_max_up); // upward max acceleration of 5m/ss
	smooth.overwriteJerkMax(0.1f); // maximum jerk of 0.1

	float dt = 0.1f; // dt is set to 0.1s

	/* It should start with acceleration */
	ut_assert_true(smooth.getIntention() == ManualIntentionZ::acceleration);

	for (int i = 0; i < 100; i++) {

		smooth.smoothVelFromSticks(vel_sp_current, dt);

		/* Test if intention is brake */
		ut_assert_true(smooth.getIntention() == ManualIntentionZ::brake);

		/* we should always use upward acceleration */
		ut_assert_true((smooth.getMaxAcceleration() - acc_max_up < FLT_EPSILON));

		/* New setpoint has to be lower than previous setpoint (NED frame) or equal 0. 0 velocity
		 * occurs once the vehicle is at perfect rest. */
		ut_assert_true((vel_sp_current < vel_sp_previous) || (fabsf(vel_sp_current) < FLT_EPSILON));


		/* We reset the previou setpoint to newest setpoint
		 * and set the current setpoint to 0 because we still want to brake.
		 * We also set vel to previous setpoint where we make the assumption that
		 * the vehicle can perfectly track the setpoints.
		 */
		vel_sp_previous = vel_sp_current;
		vel_sp_current = 0.0f;
		vel = vel_sp_previous;


	}

	return true;
}

bool SmoothZTest::brakeDownward()
{
	/* Downward flight and want to stop */
	float stick_current = 0.0f; // sticks are at zero position
	float vel_sp_current = 0.0f; // desired velocity is 0m/s
	float vel_sp_previous = -5.0f; // the demanded previous setpoint was -5m/s downwards
	float vel = vel_sp_previous; // assume that current velocity is equal to previous vel setpoint
	float acc_max_up = 5.0f;
	float acc_max_down = 2.0f;

	ManualSmoothingZ smooth(nullptr, vel, stick_current);

	/* overwrite parameters since they might change depending on configuration */
	smooth.overwriteAccelerationDown(acc_max_down); // downward max acceleration of 2m/ss
	smooth.overwriteAccelerationUp(acc_max_up); // upward max acceleration of 5m/ss
	smooth.overwriteJerkMax(0.1f); // maximum jerk of 0.1

	float dt = 0.1f; // dt is set to 0.1s

	/* It should start with acceleration */
	ut_assert_true(smooth.getIntention() == ManualIntentionZ::acceleration);

	for (int i = 0; i < 100; i++) {

		smooth.smoothVelFromSticks(vel_sp_current, dt);

		/* Test if intention is brake */
		ut_assert_true(smooth.getIntention() == ManualIntentionZ::brake);

		/* New setpoint has to be larger than previous setpoint (NED frame) or equal 0. 0 velocity
		 * occurs once the vehicle is at perfect rest. */
		ut_assert_true((vel_sp_current > vel_sp_previous) || (fabsf(vel_sp_current) < FLT_EPSILON));

		/* we should always use downward acceleration except when vehicle is at rest*/
		if (fabsf(vel_sp_previous) < FLT_EPSILON) {
			ut_assert_true(fabsf(smooth.getMaxAcceleration() - acc_max_up) < FLT_EPSILON);

		} else {
			ut_assert_true(fabsf(smooth.getMaxAcceleration() - acc_max_down) < FLT_EPSILON);
		}


		/* We reset the previou setpoint to newest setpoint
		 * and set the current setpoint to 0 because we still want to brake.
		 * We also set vel to previous setpoint where we make the assumption that
		 * the vehicle can perfectly track the setpoints.
		 */
		vel_sp_previous = vel_sp_current;
		vel_sp_current = 0.0f;
		vel = vel_sp_previous;

	}

	return true;
}

bool SmoothZTest::accelerateUpwardFromBrake()
{
	/* Downward flight and want to stop */
	float stick_current = -1.0f; // sticks are at full upward position
	float vel_sp_target = -5.0f; // desired velocity is at -5m/s
	float vel_sp_current = vel_sp_target;
	float vel_sp_previous = 0.0f; // the demanded previous setpoint was 0m/s downwards
	float vel = vel_sp_previous; // assume that current velocity is equal to previous vel setpoint
	float acc_max_up = 5.0f;
	float acc_max_down = 2.0f;

	ManualSmoothingZ smooth(nullptr, vel, stick_current);

	/* overwrite parameters since they might change depending on configuration */
	smooth.overwriteAccelerationDown(acc_max_down); // downward max acceleration of 2m/ss
	smooth.overwriteAccelerationUp(acc_max_up); // upward max acceleration of 5m/ss
	smooth.overwriteJerkMax(0.1f); // maximum jerk of 0.1

	float dt = 0.1f; // dt is set to 0.1s

	for (int i = 0; i < 100; i++) {

		smooth.smoothVelFromSticks(vel_sp_current, dt);

		/* Test if intention is acceleration */
		ut_assert_true(smooth.getIntention() == ManualIntentionZ::acceleration);

		/* we should always use upward acceleration */
		ut_assert_true(fabsf(smooth.getMaxAcceleration() - acc_max_up) < FLT_EPSILON);

		/* New setpoint has to be larger than previous setpoint or equal to target velocity
		 * vel_sp_current. The negative sign is because of NED frame.
		 */
		ut_assert_true((-vel_sp_current > -vel_sp_previous) || (fabsf(vel_sp_current - vel_sp_previous) < FLT_EPSILON));


		/* We reset the previous setpoint to newest setpoint and reset the current setpoint.
		 * We also set the current velocity to the previous setpoint with the assumption that
		 * the vehicle does perfect tracking.
		 */
		vel_sp_previous = vel_sp_current;
		vel_sp_current = vel_sp_target;
		vel = vel_sp_previous;

	}

	return true;
}

bool SmoothZTest::accelerateDownwardFromBrake()
{
	/* Downward flight and want to stop */
	float stick_current = 1.0f; // sticks are at full downward position
	float vel_sp_target =  5.0f; // desired velocity is at 5m/s
	float vel_sp_current = vel_sp_target;
	float vel_sp_previous = 0.0f; // the demanded previous setpoint was 0m/s downwards
	float vel = vel_sp_previous; // assume that current velocity is equal to previous vel setpoint
	float acc_max_up = 5.0f;
	float acc_max_down = 2.0f;

	ManualSmoothingZ smooth(nullptr, vel, stick_current);

	/* overwrite parameters since they might change depending on configuration */
	smooth.overwriteAccelerationDown(acc_max_down); // downward max acceleration of 2m/ss
	smooth.overwriteAccelerationUp(acc_max_up); // upward max acceleration of 5m/ss
	smooth.overwriteJerkMax(0.1f); // maximum jerk of 0.1

	float dt = 0.1f; // dt is set to 0.1s

	for (int i = 0; i < 100; i++) {

		smooth.smoothVelFromSticks(vel_sp_current, dt);

		/* Test if intention is acceleration */
		ut_assert_true(smooth.getIntention() == ManualIntentionZ::acceleration);

		/* we should always use downward acceleration except when target velocity is reached */
		if (fabsf(vel_sp_current - vel_sp_target) < FLT_EPSILON) {
			ut_assert_true(smooth.getMaxAcceleration() - acc_max_up < FLT_EPSILON);

		} else {
			ut_assert_true(fabsf(smooth.getMaxAcceleration() - acc_max_down) < FLT_EPSILON);
		}

		/* New setpoint has to be larger than previous setpoint or equal to target velocity
		 * vel_sp_current (NED frame).
		 */
		ut_assert_true((vel_sp_current > vel_sp_previous) || (fabsf(vel_sp_current - vel_sp_target) < FLT_EPSILON));


		/* We reset the previous setpoint to newest setpoint and reset the current setpoint.
		 * We also set the current velocity to the previous setpoint with the assumption that
		 * the vehicle does perfect tracking.
		 */
		vel_sp_previous = vel_sp_current;
		vel_sp_current = vel_sp_target;
		vel = vel_sp_previous;

	}

	return true;
}

ut_declare_test_c(test_smooth_z, SmoothZTest)
