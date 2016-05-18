
/**
 * Flight path tracker tests
 *
 */

#include "tracker_test.h"

#include "../tracker.h"
#include <unit_test/unit_test.h>
#include <math.h>       // sqrt
#include <cstring>      // std::strcat
#include <climits>      // INT_MAX
#include <lib/mathlib/math/Limits.hpp>  // std::min, max


#define DEFINE_TEST(testName, ...) \
	const size_t testName ## Ret[] = { __VA_ARGS__ }; \
	const int testName[] =

#define USE_TEST(testName) { .name = #testName, .path = testName, .path_size = sizeof(testName) / sizeof(testName[0]), .ret = testName ## Ret, .ret_size = sizeof(testName ## Ret) / sizeof(testName ## Ret[0]) }


DEFINE_TEST(noLoop, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
{
#include "no_loop.path"
};

DEFINE_TEST(simpleLoop, 8, 1, 0)
{
#include "simple_loop.path"
};

// aka:                      21               9  31      13       2
DEFINE_TEST(complexLoop, 50, 49, 20, 19, 18, 17, 10, 30, 29, 28, 27, 1, 0)
{
#include "complex_loop.path"
};

// aka:                      8
// aka:                     29                   2
// aka:                     36                  16
DEFINE_TEST(largeNodes, 48, 47, 28, 27, 26, 25, 24, 1, 0)
{
#include "large_nodes.path"
};

// aka:                                          14
DEFINE_TEST(fromSim, 48, 47, 46, 45, 44, 43, 42, 41, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
{
#include "from_sim.path"
};

// aka:
DEFINE_TEST(simpleJumps, 25, 24, 4, 3, 2, 1, 0)
{
#include "simple_jumps.path"
};


#ifdef TRACKER_TEST_LONG_PATHS
DEFINE_TEST(longPath1)
{
#include "long_path1.path"
};

DEFINE_TEST(longPath2)
{
#include "long_path2.path"
};
#endif


class TrackerTest : public UnitTest
{
public:
	TrackerTest();
	virtual ~TrackerTest();

	bool run_tests(void);

private:

	struct line_test_t {
		Tracker::ipos_t delta1, end1;
		Tracker::ipos_t delta2, end2;
		Tracker::ipos_t result;
	};

	struct test_t {
		const char *name;
		const int *path;
		const size_t path_size;
		const size_t *ret;
		const size_t ret_size;
	};

	struct return_state_t {
		const test_t *test;
		size_t target_index;
		int no_progress;
		float x, y, z;
	};

	float get_distance(float x1, float y1, float z1, float x2, float y2, float z2);

	// Returns the distance of the specified point to a line in the specified test
	float distance_to_line(float x, float y, float z, const test_t *test, size_t line_end_index);

	// Registers the progress represented by the new position.
	// Returns false if the new position is no longer on the expected return path.
	bool advance_state(return_state_t &state, float new_x, float new_y, float new_z, float accuracy);

	// Returns true if the return state has reached the home position
	bool detect_completion(return_state_t &state);

	// Initializes the return path supervision from any given position
	return_state_t init_return_state(float x, float y, float z, const test_t *test);

	// Makes the tracker return along the shortest path home and tests whether it chooses the right path.
	// Returns false if the test fails and fills msg with an appropriate error message.
	bool try_return_supervised(Tracker &tracker, const test_t *test, char *msg);

	// Makes the tracker return along the shortest path home. Use this if there is no well known return path.
	// Returns false if the test fails and fills msg with an appropriate error message.
	bool try_return_unsupervised(Tracker &tracker, const test_t *test, float home_x, float home_y, float home_z,
				     float home_to_path_dist, char *msg);

	// Simulates a flight along the defined test path,
	// and then returns on the shortest path while verifying that
	// the tracker recommends the expected return path.
	bool fly_and_return_test();

	// Simulates a flight along the defined test path,
	// returns half of the shortest path, flies to some other pseudorandom (deterministic)
	// position in the graph, and then returns along the shortest path from there.
	bool fly_and_leave_return_path_test();

	// Simulates flight along the defined test path, selects some pseudorandom
	// new home along the path and returns to that new home
	bool fly_and_change_home_test();

	// Takes several performance measurements
	bool performance_test();

	// Tests if the minimum line-to-line delta is calculated correctly
	bool line_to_line_test();

	// Enables graph rewriting in some tests
	bool rewrite_graph = false;

	Tracker _tracker;

	// An array of flight paths and their correct return paths
	// We can do various tests on each of these.
	static const test_t test_cases[];

	// An array of line pairs and the shortest delta between them.
	static line_test_t line_test_cases[];
};

const TrackerTest::test_t TrackerTest::test_cases[] = {
	USE_TEST(noLoop),
	USE_TEST(simpleLoop),
	USE_TEST(complexLoop),
	USE_TEST(largeNodes),
	USE_TEST(fromSim),
	USE_TEST(simpleJumps),
#ifdef TRACKER_TEST_LONG_PATHS
	USE_TEST(longPath1),
	USE_TEST(longPath2)
#endif
};

TrackerTest::line_test_t TrackerTest::line_test_cases[] = {
	// test cases generated with Matlab script Tools/Matlab/lineToLineTest.m
	{ .delta1 = { 0, 0, 0 }, .end1 = { 0, 0, 0 }, .delta2 = { 0, 0, 0 }, .end2 = { 0, 0, 0 }, .result = { 0, 0, 0 } },
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { 6, 2, 2 }, .end2 = { 2, 3, 0 }, .result = { -1, 2, 1 } },
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { 6, 2, 2 }, .end2 = {5,  4, 4 }, .result = { 0, 2, 3 } },
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 2, -2 }, .delta2 = { 4, 1, 1 }, .end2 = {5,  4, 4 }, .result = { -1, 1, 5 } },
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { 3, 1, 1 }, .end2 = {-1, 2, -2 }, .result = { -1, 2, 0 } },
	//{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { -2, 3, 2 }, .end2 = {-1, 2, -2 }, .result = { 0, 1, -1 } }, with rounding temp_delta
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { -2, 3, 2 }, .end2 = {-1, 2, -2 }, .result = { -1, 0, -1 } }, // without rounding temp_delta
	{ .delta1 = { 4, 2, -1 }, .end1 = { 2, 1, -2 }, .delta2 = { -2, 3, 2 }, .end2 = {-3, 5, 0 }, .result = { -1, 2, 0 } },
	{ .delta1 = { 4, 2, -1 }, .end1 = { 6, 3, -3 }, .delta2 = { -2, 3, 2 }, .end2 = {-3, 5, 0 }, .result = { -3, 1, 0 } }
};


TrackerTest::TrackerTest()
{
}

TrackerTest::~TrackerTest()
{
}


float TrackerTest::get_distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}


float TrackerTest::distance_to_line(float x, float y, float z, const test_t *test, size_t line_end_index)
{
	float line_end_x = test->path[test->ret[line_end_index] * 3];
	float line_end_y = test->path[test->ret[line_end_index] * 3 + 1];
	float line_end_z = test->path[test->ret[line_end_index] * 3 + 2];
	float base_x = test->path[test->ret[line_end_index - 1] * 3] - line_end_x;
	float base_y = test->path[test->ret[line_end_index - 1] * 3 + 1] - line_end_y;
	float base_z = test->path[test->ret[line_end_index - 1] * 3 + 2] - line_end_z;
	float span_x = x - line_end_x;
	float span_y = y - line_end_y;
	float span_z = z - line_end_z;

	float cross_x = base_y * span_z - base_z * span_y;
	float cross_y = base_z * span_x - base_x * span_z;
	float cross_z = base_x * span_y - base_y * span_x;

	return sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z) / sqrt(base_x * base_x + base_y * base_y + base_z
			* base_z);
}


bool TrackerTest::advance_state(return_state_t &state, float new_x, float new_y, float new_z, float accuracy)
{
	size_t target_index = state.target_index;
	state.no_progress++;

	do {
		float target_x = state.test->path[state.test->ret[target_index] * 3];
		float target_y = state.test->path[state.test->ret[target_index] * 3 + 1];
		float target_z = state.test->path[state.test->ret[target_index] * 3 + 2];

		float old_dist_to_target = get_distance(target_x, target_y, target_z, state.x, state.y, state.z);
		float new_dist_to_target = get_distance(target_x, target_y, target_z, new_x, new_y, new_z);

		if (new_dist_to_target <= old_dist_to_target) {
			if (new_dist_to_target < old_dist_to_target) {
				state.no_progress = 0;
			}

			if (distance_to_line(new_x, new_y, new_z, state.test, target_index) <= accuracy) {
				state.target_index = target_index;
				state.x = new_x;
				state.y = new_y;
				state.z = new_z;
				return true;
			}
		}
	} while (++target_index < state.test->ret_size);

	return false;
}


bool TrackerTest::detect_completion(return_state_t &state)
{
	float home_x = state.test->path[0];
	float home_y = state.test->path[1];
	float home_z = state.test->path[2];

	float dist_to_home = get_distance(state.x, state.y, state.z, home_x, home_y, home_z);
	return dist_to_home < 2 * Tracker::GRID_SIZE; // tracking accuracy at home is twice the grid size
}


TrackerTest::return_state_t TrackerTest::init_return_state(float x, float y, float z, const test_t *test)
{
	return {
		.test = test,
		.target_index = 0,
		.no_progress = 0,
		.x = x,
		.y = y,
		.z = z
	};
}


#define inner_assert(value, ...) \
	do { \
		if (!(value)) { \
			sprintf(msg, __VA_ARGS__); \
			std::strcat(msg, ", on test "); \
			std::strcat(msg, test->name); \
			return false; \
		} \
	} while (0)


bool TrackerTest::try_return_supervised(Tracker &tracker, const test_t *test, char *msg)
{
	// Return along the shortest path while checking if it's what we expect

	Tracker::path_finding_context_t context;
	float x, y, z;
	inner_assert(tracker.init_return_path(context, x, y, z), "tracker could not init return path");
	return_state_t state = init_return_state(x, y, z, test);

	for (;;) {

		if (!tracker.advance_return_path(context, x, y, z)) {
			break;
		}

		size_t target = test->ret[state.target_index];
		inner_assert(!tracker.get_graph_fault(), "graph inconsistent");
		float accuracy = tracker.get_accuracy_at(x, y, z);
		inner_assert(accuracy <= 5, "tracker gave an unsatisfying accuracy guarantee (%f)",
			     (double)accuracy); // arbitrarily chosen
		inner_assert(advance_state(state, x, y, z, accuracy),
			     "vehicle diverted from expected return path: (%f, %f, %f) is not towards target %d %d %d", (double)x, (double)y,
			     (double)z, test->path[target * 3], test->path[target * 3 + 1], test->path[target * 3 + 2]);
		inner_assert(state.no_progress < TRACKER_MAX_NO_PROGRESS,
			     "no progress detected from (%f, %f, %f) to (%f, %f, %f), target is %d %d %d", (double)state.x, (double)state.y,
			     (double)state.z, (double)x, (double)y, (double)z, test->path[target * 3], test->path[target * 3 + 1],
			     test->path[target * 3 + 2]);

#ifdef TRACKER_TEST_LOOKAHEAD
		// Prefetch the return path starting at the current position
		Tracker::path_finding_context_t inner_context;
		float inner_x, inner_y, inner_z;
		inner_assert(tracker.init_return_path(inner_context, inner_x, inner_y, inner_z), "tracker could not init return path");
		return_state_t inner_state = init_return_state(inner_x, inner_y, inner_z, test);

		while (tracker.advance_return_path(inner_context, inner_x, inner_y, inner_z)) {
			inner_assert(!tracker.get_graph_fault(), "graph inconsistent");
			accuracy = tracker.get_accuracy_at(inner_x, inner_y, inner_z);
			inner_assert(accuracy <= 5, "tracker gave an unsatisfying accuracy guarantee (%f)",
				     (double)accuracy); // arbitrarily chosen
			inner_assert(advance_state(inner_state, inner_x, inner_y, inner_z, accuracy),
				     "look-ahead return path diverted from expected return path");
			inner_assert(inner_state.no_progress < TRACKER_MAX_NO_PROGRESS,
				     "no progress detected in look-ahead return path from (%f, %f, %f) to (%f, %f, %f), target index is %zu",
				     (double)inner_state.x, (double)inner_state.y, (double)inner_state.z, (double)inner_x, (double)inner_y, (double)inner_z,
				     inner_state.target_index);
		}

		inner_assert(detect_completion(inner_state),
			     "look-ahead return path did not reach home, ended at (%f, %f, %f), target index is %zu", (double)inner_state.x,
			     (double)inner_state.y, (double)inner_state.z, inner_state.target_index);
#endif

		// Follow the return path by one position
		TRACKER_DBG("advance to (%.2f %.2f %.2f)", (double)x, (double)y, (double)z);
		tracker.update(x, y, z);
	}

	inner_assert(!tracker.get_graph_fault(), "graph inconsistent");
	inner_assert(detect_completion(state), "vehicle did not reach home, currently at (%f, %f, %f), home is (%d, %d, %d)",
		     (double)state.x, (double)state.y, (double)state.z, test->path[0], test->path[1], test->path[2]);

	return true;
}


bool TrackerTest::try_return_unsupervised(Tracker &tracker, const test_t *test, float home_x, float home_y,
		float home_z, float home_to_path_dist, char *msg)
{
	int steps = 0;
	float x, y, z;
	Tracker::path_finding_context_t context;

	// As long as we're not home, return along the proposed path
	inner_assert(tracker.init_return_path(context, x, y, z), "tracker could not init return path");

	while (tracker.advance_return_path(context, x, y, z)) {
		tracker.update(x, y, z);
		inner_assert(!tracker.get_graph_fault(), "graph inconsistent");
		inner_assert(steps++ < 256, "return-to-home did take too many steps"); // make sure the loop terminates
		TRACKER_DBG("return from %d, %d, %d, home is %.3f, %.3f, %.3f", (int)x, (int)y, (int)z, (double)home_x, (double)home_y,
			    (double)home_z);
	}

	inner_assert(!tracker.get_graph_fault(), "graph inconsistent");

	// Check if we're actually home
	inner_assert(get_distance(x, y, z, home_x, home_y,
				  home_z) <= 2 * Tracker::GRID_SIZE + home_to_path_dist, // accuracy at home is twice the grid size
		     "the vehicle didn't return home: it went to (%.2f %.2f %.2f) while home is at (%.2f %.2f %.2f)",
		     (double)x, (double)y, (double)z, (double)home_x, (double)home_y, (double)home_z);

	return true;
}


bool TrackerTest::fly_and_return_test(void)
{
	for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
		const test_t *test = test_cases + t;

		if (!test->ret_size) {
			continue;
		}

		TRACKER_DBG("running fly-and-return on %s %s", test->name,
			    rewrite_graph ? "with graph rewriting" : "without graph rewriting");

		_tracker.reset_graph();
		_tracker.set_home(test->path[0], test->path[1], test->path[2]);

		// Simulate flight along the specified path
		for (size_t p = 0; p < test->path_size; p += 3) {
			_tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
		}


#ifdef DEBUG_TRACKER
		_tracker.dump_graph();
#endif

		// Rewrite graph to contain nothing but the return path
		if (rewrite_graph) {
			_tracker.rewrite_graph();

#ifdef DEBUG_TRACKER
			_tracker.dump_graph();
#endif
		}

		// Return home
		char msg[1024];

		if (!try_return_supervised(_tracker, test, msg)) {
			ut_assert(msg, false);
		}
	}

	return true;
}


bool TrackerTest::fly_and_leave_return_path_test(void)
{
	for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
		const test_t *test = test_cases + t;
		TRACKER_DBG("running fly-and-leave-return-path on %s", test->name);

		_tracker.reset_graph();
		_tracker.set_home(test->path[0], test->path[1], test->path[2]);

		// Simulate flight along the specified path
		for (size_t p = 0; p < test->path_size; p += 3) {
			_tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
		}

		int x = 0, y = 0, z = 0;

		// Follow half of the return path
		for (size_t r = 0; r < test->ret_size / 2; r++) {
			x = test->path[test->ret[r] * 3];
			y = test->path[test->ret[r] * 3 + 1];
			z = test->path[test->ret[r] * 3 + 2];
			_tracker.update(x, y, z);
		}

		// Select a destination using a deterministic pseudorandom index
		size_t dest_index = (x + y + z + test->ret[test->ret_size / 2]) % (test->path_size / 3);
		int dest_x = test->path[dest_index * 3];
		int dest_y = test->path[dest_index * 3 + 1];
		int dest_z = test->path[dest_index * 3 + 2];

		// Fly to the specified index (todo: when the tracker's delta limit gets fixed, we can directly jump to the destination).
		while (x != dest_x || y != dest_y || z != dest_z) {
			x += math::min(15, math::max(-16, dest_x - x));
			y += math::min(15, math::max(-16, dest_y - y));
			z += math::min(15, math::max(-16, dest_z - z));
			_tracker.update(x, y, z);
		}

#ifdef DEBUG_TRACKER
		_tracker.dump_graph();
#endif

		// Return home
		char msg[512];

		if (!try_return_unsupervised(_tracker, test, test->path[0], test->path[1], test->path[2], 0, msg)) {
			ut_assert(msg, false);
		}
	}

	return true;
}


bool TrackerTest::fly_and_change_home_test(void)
{
	for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
		const test_t *test = test_cases + t;
		TRACKER_DBG("running fly-and-change-home on %s", test->name);

		_tracker.reset_graph();
		_tracker.set_home(test->path[0], test->path[1], test->path[2]);

		// Simulate flight along the specified path
		for (size_t p = 0; p < test->path_size; p += 3) {
			_tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
		}

		// Select a home using a deterministic pseudorandom index
		size_t dest_index = (test->path[0] + test->path[1] + test->path[2] + test->ret[test->ret_size / 2]) %
				    (test->path_size / 3);
		int home_x = test->path[dest_index * 3];
		int home_y = test->path[dest_index * 3 + 1] + Tracker::GRID_SIZE; // add a little bit of offset
		int home_z = test->path[dest_index * 3 + 2] + Tracker::GRID_SIZE; // add a little bit of offset

		_tracker.set_home(home_x, home_y, home_z);

#ifdef DEBUG_TRACKER
		_tracker.dump_graph();
#endif

		// Return home
		char msg[512];

		if (!try_return_unsupervised(_tracker, test, home_x, home_y, home_z, (double)sqrt(2) * (double)Tracker::GRID_SIZE,
					     msg)) {
			ut_assert(msg, false);
		}
	}

	return true;
}


bool TrackerTest::performance_test(void)
{
	for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
		const test_t *test = test_cases + t;
		PX4_WARN("running performance test on %s", test->name);

		_tracker.reset_graph();
		_tracker.set_home(test->path[0], test->path[1], test->path[2]);

		// Simulate flight along the specified path
		for (size_t p = 0; p < test->path_size; p += 3) {
			_tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
		}



		char number[128] = { 0 };
		char msg1[256] = { 0 };
		char msg2[256] = { 0 };

		// print stats of each performance measurement
		for (int i = 0; i < _tracker.memory_pressure - 1; i++) {
			Tracker::compress_perf_t *perf = _tracker.perf_measurements + i;

			sprintf(number, "%zu, %zu; %zu, %zu; ", perf->deltas_before, perf->nodes_before, perf->deltas_after, perf->nodes_after);
			std::strcat(msg1, number);

			sprintf(number, "%.3f ", (double)perf->runtime / (double)1e3f);
			std::strcat(msg2, number);
		}

		// print final memory stats
		sprintf(number, "%zu, %zu; 0, 0", _tracker.graph_next_write, _tracker.node_count);
		std::strcat(msg1, number);

		_tracker.dump_graph();
		PX4_WARN("memory pressure: %d", _tracker.memory_pressure);
		PX4_WARN("memory usage: [%s]", msg1);
		PX4_WARN("CPU usage: [%s]", msg2);
	}

	return true;
}


bool TrackerTest::line_to_line_test()
{
	size_t count = sizeof(line_test_cases) / sizeof(line_test_cases[0]);

	for (size_t i = 0; i < 2 * count; i++) {
		// We do each test twice, the second time we swap the lines
		line_test_t *t = line_test_cases + (i % count);

		if (i >= count) {
			Tracker::ipos_t temp = t->delta1;
			t->delta1 = t->delta2;
			t->delta2 = temp;
			temp = t->end1;
			t->end1 = t->end2;
			t->end2 = temp;
			t->result = -t->result;
		}

		int coef1, coef2;
		Tracker::ipos_t delta = Tracker::get_line_to_line_delta(t->delta1, t->end1, t->delta2, t->end2, coef1, coef2, false,
					false);

		Tracker::ipos_t p1 = t->end1 - Tracker::apply_coef(t->delta1, coef1);
		Tracker::ipos_t p2 = t->end2 - Tracker::apply_coef(t->delta2, coef2);
		Tracker::ipos_t implied_delta = p2 - p1;

		char msg[512];
		sprintf(msg,
			"(%d %d %d) - %.2f * (%d %d %d) = (%d %d %d), (%d %d %d) - %.2f * (%d %d %d) = (%d %d %d), delta should be (%d %d %d) but have (%d %d %d)",
			t->end1.x, t->end1.y, t->end1.z, (double)Tracker::coef_to_float(coef1), t->delta1.x, t->delta1.y, t->delta1.z, p1.x,
			p1.y, p1.z,
			t->end2.x, t->end2.y, t->end2.z, (double)Tracker::coef_to_float(coef2), t->delta2.x, t->delta2.y, t->delta2.z, p2.x,
			p2.y, p2.z,
			t->result.x, t->result.y, t->result.z, delta.x, delta.y, delta.z);

		ut_compare(msg, implied_delta.x, delta.x);
		ut_compare(msg, implied_delta.y, delta.y);
		ut_compare(msg, implied_delta.z, delta.z);
		ut_compare(msg, delta.x, t->result.x);
		ut_compare(msg, delta.y, t->result.y);
		ut_compare(msg, delta.z, t->result.z);
	}

	return true;
}


bool TrackerTest::run_tests(void)
{
	ut_run_test(line_to_line_test);

	rewrite_graph = false;
	ut_run_test(fly_and_return_test);
	ut_run_test(fly_and_leave_return_path_test);
	ut_run_test(fly_and_change_home_test);

	rewrite_graph = true;
	ut_run_test(fly_and_return_test);
	ut_run_test(fly_and_leave_return_path_test);
	ut_run_test(fly_and_change_home_test);

	// These tests have nothing to do with correctness, but we can use them to asses performance of the tracker.
	//ut_run_test(performance_test);

	return (_tests_failed == 0);
}

ut_declare_test(trackerTest, TrackerTest)
