#include <gtest/gtest.h>

#include <motion_planning/PositionSmoothing.hpp>

TEST(PositionSmoothingBasicTest, AllZeroCase)
{
	PositionSmoothing position_smoothing;
	PositionSmoothing::PositionSmoothingSetpoints out;

	position_smoothing.generateSetpoints(
		Vector3f(),
	{Vector3f(), Vector3f(), Vector3f()},
	Vector3f(),
	0.f,
	false,
	out
	);

	EXPECT_EQ(out.jerk, Vector3f());
	EXPECT_EQ(out.acceleration, Vector3f());
	EXPECT_EQ(out.velocity, Vector3f());
	EXPECT_EQ(out.position, Vector3f());
	EXPECT_EQ(out.unsmoothed_velocity, Vector3f());
}

static constexpr float MAX_JERK = 4.f;
static constexpr float MAX_ACCELERATION = 3.f;

static constexpr float MAX_ALLOWED_HOR_ERR = 2.f;
static constexpr float VERTICAL_ACCEPTANCE_RADIUS = 0.8f;
static constexpr float CRUISE_SPEED = 5.f;
static constexpr float MAX_VELOCITY = CRUISE_SPEED;

static constexpr float HORIZONTAL_TRAJECTORY_GAIN = 0.5f;
static constexpr float TARGET_ACCEPTANCE_RADIUS = 0.5f;


class PositionSmoothingTest : public ::testing::Test
{

public:
	PositionSmoothing _position_smoothing;

	PositionSmoothingTest()
	{
		_position_smoothing.setMaxJerk(MAX_JERK);
		_position_smoothing.setMaxAcceleration({MAX_ACCELERATION, MAX_ACCELERATION, MAX_ACCELERATION});
		_position_smoothing.setMaxVelocity({MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY});
		_position_smoothing.setMaxAllowedHorizontalError(MAX_ALLOWED_HOR_ERR);
		_position_smoothing.setVerticalAcceptanceRadius(VERTICAL_ACCEPTANCE_RADIUS);
		_position_smoothing.setCruiseSpeed(CRUISE_SPEED);
		_position_smoothing.setHorizontalTrajectoryGain(HORIZONTAL_TRAJECTORY_GAIN);
		_position_smoothing.setTargetAcceptanceRadius(TARGET_ACCEPTANCE_RADIUS);

		_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
	}

	static void expectDynamicsLimitsRespected(const PositionSmoothing::PositionSmoothingSetpoints &setpoints)
	{
		EXPECT_LE(fabsf(setpoints.velocity(0)), MAX_VELOCITY) << "Velocity in x too high\n";
		EXPECT_LE(fabsf(setpoints.velocity(1)), MAX_VELOCITY) << "Velocity in y too high\n";
		EXPECT_LE(fabsf(setpoints.velocity(2)), MAX_VELOCITY) << "Velocity in z too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(0)), MAX_ACCELERATION) << "Acceleration in x too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(1)), MAX_ACCELERATION) << "Acceleration in y too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(2)), MAX_ACCELERATION) << "Acceleration in z too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(0)), MAX_JERK) << "Jerk in x too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(1)), MAX_JERK) << "Jerk in y too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(2)), MAX_JERK) << "Jerk in z too high\n";
	}
};


TEST_F(PositionSmoothingTest, reachesTargetPositionSetpoint)
{
	const int N_ITER = 2000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f FF_VELOCITY{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};

	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, TARGET};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			FF_VELOCITY,
			DELTA_T,
			false,
			out
		);
		position = out.position;
		expectDynamicsLimitsRespected(out);

		if (position == TARGET) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	EXPECT_EQ(TARGET, position);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}


TEST_F(PositionSmoothingTest, reachesTargetVelocityIntegration)
{
	const int N_ITER = 2000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f FF_VELOCITY{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};

	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, TARGET};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			FF_VELOCITY,
			DELTA_T,
			false,
			out
		);
		position += out.velocity * DELTA_T;
		expectDynamicsLimitsRespected(out);


		if (position == TARGET) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	EXPECT_EQ(TARGET, position);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}


TEST_F(PositionSmoothingTest, reachesTargetInitialVelocity)
{
	const int N_ITER = 20000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};
	const Vector3f NEXT_TARGET{8.f, 12.f, 80.f};

	const float XY_ACC_RAD = 10.f;
	const float Z_ACC_RAD = 0.8f;


	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, TARGET};
	Vector3f ff_velocity{1.f, 0.1f, 0.3f};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			ff_velocity,
			DELTA_T,
			false,
			out
		);
		position = out.position;
		ff_velocity = {0.f, 0.f, 0.f};
		expectDynamicsLimitsRespected(out);

		if (Vector2f(position.xy() - TARGET.xy()).norm() < XY_ACC_RAD && fabsf(position(2) - TARGET(2)) < Z_ACC_RAD) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	EXPECT_LT(Vector2f(position.xy() - TARGET.xy()).norm(), XY_ACC_RAD);
	EXPECT_LT(fabsf(position(2) - TARGET(2)), Z_ACC_RAD);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}


// Fly through a waypoint whose triplet never advances (e.g. an overshoot the navigator
// can't accept). The look-ahead point must not keep marching down the extended leg: the
// vehicle has to brake and come back to the target instead of drifting away forever.
TEST_F(PositionSmoothingTest, doesNotDriftPastUnreachedWaypoint)
{
	const int N_ITER = 3000; // 60 s at 50 Hz
	const float DELTA_T = 0.02f;

	const Vector3f PREV{0.f, 0.f, 0.f};
	const Vector3f TARGET{20.f, 0.f, 0.f};
	const Vector3f NEXT{40.f, 2.f, 0.f}; // near-collinear next leg -> high corner speed -> clear fly-through

	// Triplet stays fixed for the whole run: the navigator never advances past TARGET.
	Vector3f waypoints[3] = {PREV, TARGET, NEXT};

	const Vector3f u_leg = (TARGET - PREV).unit_or_zero();
	const float leg_length = (TARGET - PREV).length();

	Vector3f position{0.f, 0.f, 0.f};
	PositionSmoothing::PositionSmoothingSetpoints out;

	bool reached_target = false;
	bool came_back = false;
	float max_distance_past_target = 0.f;

	for (int i = 0; i < N_ITER; i++) {
		_position_smoothing.generateSetpoints(position, waypoints, Vector3f{}, DELTA_T, false, out);
		position = out.position;
		expectDynamicsLimitsRespected(out);

		const float along_track = Vector3f(position - PREV) * u_leg;

		if (Vector3f(position - TARGET).length() < 1.f) {
			reached_target = true;
		}

		if (reached_target) {
			max_distance_past_target = fmaxf(max_distance_past_target, along_track - leg_length);

			// Once the vehicle has gone past the target, the fix must turn it around.
			if (along_track > leg_length + 1.f && out.velocity * u_leg < -0.1f) {
				came_back = true;
			}
		}
	}

	EXPECT_TRUE(reached_target) << "Vehicle never reached the target waypoint\n";
	EXPECT_TRUE(came_back) << "Vehicle never turned back toward the unreached waypoint (it drifted away)\n";
	// Without the fix the look-ahead marches down the extended leg and this grows unbounded.
	EXPECT_LT(max_distance_past_target, 10.f) << "Vehicle drifted too far past the unreached waypoint\n";
}
