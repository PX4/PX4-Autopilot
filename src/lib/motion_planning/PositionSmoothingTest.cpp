#include <gtest/gtest.h>

#include <motion_planning/PositionSmoothing.hpp>
#include <mathlib/mathlib.h>
#include <vector>
#include <utility>
#include <random>

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

TEST_F(PositionSmoothingTest, smallAltitudeErrorDoesNotCommandFullVerticalSpeed)
{
	const Vector3f start{-208.9f, -689.5f, -59.f};
	const Vector3f target{-97.f, -71.6f, -59.f};
	const Vector2f direction = Vector2f(target - start).unit_or_zero();

	for (float next_height_change : {-41.f, 41.f}) {
		const Vector3f waypoints[3] = {start, target, {12.15f, -50.6f, target(2) + next_height_change}};

		for (float error : {-0.01f, -0.001f, 0.f, 0.001f, 0.01f}) {
			SCOPED_TRACE(error);
			const Vector3f position = start + Vector3f{0.f, 0.f, error};
			_position_smoothing.reset({}, {direction(0) * CRUISE_SPEED, direction(1) * CRUISE_SPEED, 0.f}, position);
			PositionSmoothing::PositionSmoothingSetpoints out;
			_position_smoothing.generateSetpoints(position, waypoints, {NAN, NAN, NAN}, 0.02f, false, out);

			// A millimetre-scale height error hundreds of metres from the waypoint must not
			// request the nonzero vertical arrival-speed limit for the following leg.
			EXPECT_LT(fabsf(out.unsmoothed_velocity(2)), 0.001f);
			EXPECT_LE(out.unsmoothed_velocity(2) * error, 0.f);
			EXPECT_NEAR(Vector2f(out.unsmoothed_velocity).norm(), CRUISE_SPEED, 1e-4f);

			if (fabsf(error) < FLT_EPSILON) {
				EXPECT_FLOAT_EQ(out.unsmoothed_velocity(2), 0.f);
			}
		}
	}
}

TEST_F(PositionSmoothingTest, levelCruiseBeforeAltitudeChangeDoesNotOscillate)
{
	// Geometry and limits from the two eastbound flight legs. Perfect position tracking
	// isolates the trajectory generator from the aircraft and position controller.
	const Vector3f start{-208.9f, -689.5f, -59.f};
	const Vector3f target{-97.f, -71.6f, -59.f};
	const Vector2f direction = Vector2f(target - start).unit_or_zero();
	const float dt = 0.02f;
	_position_smoothing.setMaxJerk(3.f);
	_position_smoothing.setMaxAccelerationXY(3.f);
	_position_smoothing.setMaxVelocityXY(12.f);
	_position_smoothing.setMaxAllowedVerticalError(1.f);
	_position_smoothing.setTargetAcceptanceRadius(2.f);

	for (float cruise_speed : {7.f, 9.f}) {
		for (float next_height_change : {-41.f, 41.f}) {
			for (float error : {-0.01f, 0.01f}) {
				SCOPED_TRACE(cruise_speed);
				SCOPED_TRACE(next_height_change);
				SCOPED_TRACE(error);
				const Vector3f waypoints[3] = {start, target, {12.15f, -50.6f, target(2) + next_height_change}};
				Vector3f position = start + Vector3f{0.f, 0.f, error};
				_position_smoothing.setCruiseSpeed(cruise_speed);
				_position_smoothing.reset({}, {direction(0) * cruise_speed, direction(1) * cruise_speed, 0.f}, position);
				float unsmoothed_vz = 0.f;
				float max_height_error = 0.f;
				float max_cruise_speed_error = 0.f;

				for (int i = 0; i < 2750; ++i) {
					// Match FlightTaskAuto's asymmetric climb/descent constraints.
					_position_smoothing.setMaxVelocityZ(unsmoothed_vz < 0.f ? 3.f : 1.5f);
					_position_smoothing.setMaxAccelerationZ(unsmoothed_vz < 0.f ? 3.f : 2.5f);
					PositionSmoothing::PositionSmoothingSetpoints out;
					_position_smoothing.generateSetpoints(position, waypoints, {NAN, NAN, NAN}, dt, false, out);
					ASSERT_TRUE(out.position.isAllFinite());
					ASSERT_TRUE(out.velocity.isAllFinite());
					position = out.position;
					unsmoothed_vz = out.unsmoothed_velocity(2);
					max_height_error = math::max(max_height_error, fabsf(position(2) - target(2)));
					max_cruise_speed_error = math::max(max_cruise_speed_error,
									   fabsf(Vector2f(out.unsmoothed_velocity).norm() - cruise_speed));
				}

				EXPECT_LT(max_height_error, 0.05f);
				EXPECT_LT(max_cruise_speed_error, 0.001f);
			}
		}
	}
}

TEST_F(PositionSmoothingTest, verticalSpeedLimitDoesNotSlowHorizontalCruise)
{
	_position_smoothing.setMaxVelocityZ(0.5f);

	for (float target_z : {-100.f, 100.f}) {
		const Vector3f target{100.f, 0.f, target_z};
		const Vector3f waypoints[3] = {{0.f, 0.f, 0.f}, target, {200.f, 0.f, 2.f * target_z}};
		_position_smoothing.reset({}, {CRUISE_SPEED, 0.f, 0.f}, {});
		PositionSmoothing::PositionSmoothingSetpoints out;
		_position_smoothing.generateSetpoints({}, waypoints, {NAN, NAN, NAN}, 0.02f, false, out);
		EXPECT_NEAR(out.unsmoothed_velocity(0), CRUISE_SPEED, 1e-4f);
		EXPECT_FLOAT_EQ(out.unsmoothed_velocity(1), 0.f);
		EXPECT_NEAR(out.unsmoothed_velocity(2), matrix::sign(target_z) * 0.5f, 1e-4f);
	}
}

TEST_F(PositionSmoothingTest, verticalWaypointsRemainReachable)
{
	_position_smoothing.setMaxVelocityZ(1.5f);

	for (float target_z : {-15.f, 15.f}) {
		const Vector3f target{0.f, 0.f, target_z};
		const Vector3f waypoints[3] = {{0.f, 0.f, 0.f}, target, target};
		Vector3f position;
		_position_smoothing.reset({}, {}, position);

		for (int i = 0; i < 2000; ++i) {
			PositionSmoothing::PositionSmoothingSetpoints out;
			_position_smoothing.generateSetpoints(position, waypoints, {NAN, NAN, NAN}, 0.02f, false, out);
			ASSERT_TRUE(out.position.isAllFinite());
			EXPECT_FLOAT_EQ(out.unsmoothed_velocity(0), 0.f);
			EXPECT_FLOAT_EQ(out.unsmoothed_velocity(1), 0.f);
			EXPECT_LE(fabsf(out.unsmoothed_velocity(2)), 1.5f);
			position = out.position;
		}

		EXPECT_NEAR(position(2), target_z, 0.01f);
	}
}

// Reproduces github.com/PX4/PX4-Autopilot/issues/28507: a sustained along-track
// velocity oscillation on a long straight AUTO.MISSION cruise leg, appearing
// only after several preceding waypoint transitions have already happened.
// Real mission waypoints (local NED, meters, relative to home), real default
// dynamic limits (MPC_JERK_AUTO=4, MPC_ACC_HOR=3), real cruise speed (10 m/s).
TEST_F(PositionSmoothingTest, wp6wp7Repro)
{
	const float DT = 0.02f;
	_position_smoothing.setMaxJerk(4.f);
	_position_smoothing.setMaxAcceleration({3.f, 3.f, 3.f});
	_position_smoothing.setMaxVelocity({10.f, 10.f, 10.f});
	_position_smoothing.setCruiseSpeed(10.f);
	_position_smoothing.setMaxAllowedHorizontalError(2.f);
	_position_smoothing.setHorizontalTrajectoryGain(0.5f);
	_position_smoothing.setTargetAcceptanceRadius(2.f);
	_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});

	const Vector3f home(0.f, 0.f, 0.f);
	const Vector3f item3(13.36f, -50.93f, 0.f);
	const Vector3f item4(412.82f, 28.79f, 0.f);
	const Vector3f item5(511.00f, -895.41f, 0.f);
	const Vector3f wp6(-209.30f, -690.78f, 0.f);
	const Vector3f wp7(-97.23f, -72.21f, 0.f);
	const Vector3f item8(12.01f, -51.22f, 0.f);

	struct Triplet { Vector3f prev, cur, next; };
	// Switch times taken from the real SITL reproduction's mission_result
	// transitions, offset so t=0 is when the takeoff/climb finished and XY
	// cruise begins (real vehicle XY velocity is ~0 at that point).
	const std::pair<float, Triplet> schedule[] = {
		{0.f,    {home, item3, item4}},
		{12.5f,  {item3, item4, item5}},
		{31.9f,  {item4, item5, wp6}},
		{75.1f,  {item5, wp6, wp7}},
		{168.8f, {wp6, wp7, item8}},
	};
	const int n_legs = sizeof(schedule) / sizeof(schedule[0]);

	Vector3f position = home;
	int sched_idx = 0;
	Triplet current = schedule[0].second;

	std::vector<float> vy_log;
	std::vector<float> t_log;
	const float T_END = 260.f;

	// Real dt comes from hrt_absolute_time() deltas (FlightTask.cpp), not a
	// perfectly fixed period -- typical scheduler jitter for a 50 Hz task.
	std::mt19937 rng(12345);
	std::uniform_real_distribution<float> jitter(-0.002f, 0.002f);

	float t = 0.f;

	for (int i = 0; i < int(T_END / DT); ++i) {
		float dt = DT + jitter(rng);

		while (sched_idx + 1 < n_legs && t >= schedule[sched_idx + 1].first) {
			sched_idx++;
			current = schedule[sched_idx].second;
		}

		Vector3f waypoints[3] = {current.prev, current.cur, current.next};
		PositionSmoothing::PositionSmoothingSetpoints out;
		_position_smoothing.generateSetpoints(position, waypoints, Vector3f(NAN, NAN, NAN), dt, false, out);
		position = out.position;
		t += dt;

		if (t >= 169.f && t <= 235.f) { // wp6->wp7 leg window
			vy_log.push_back(out.velocity(1));
			t_log.push_back(t);
		}
	}

	ASSERT_GT(vy_log.size(), 0u);
	float vmin = 1e9f, vmax = -1e9f, sum = 0.f;

	for (float v : vy_log) { vmin = math::min(vmin, v); vmax = math::max(vmax, v); sum += v; }

	float mean = sum / vy_log.size();
	float sq = 0.f;

	for (float v : vy_log) { sq += (v - mean) * (v - mean); }

	float stdv = sqrtf(sq / vy_log.size());
	printf("wp6wp7Repro: vy n=%zu min=%.3f max=%.3f range=%.3f std=%.4f\n",
	       vy_log.size(), (double)vmin, (double)vmax, (double)(vmax - vmin), (double)stdv);

	// Dump a coarse trace so the waveform shape (oscillating vs. flat) is visible.
	for (size_t i = 0; i < vy_log.size(); i += 25) {
		printf("  t=%.2f vy=%.3f\n", (double)t_log[i], (double)vy_log[i]);
	}
}
