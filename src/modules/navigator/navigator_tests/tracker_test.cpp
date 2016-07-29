
/**
 * Flight path tracker tests
 *
 */

#include "tracker_test.h"

#include "../tracker.h"
#include <unit_test/unit_test.h>
#include <cstring>      // std::strcat
#include <algorithm>    // std::min, std::max
#include <climits>    // INT_MAX


#define DEFINE_TEST(testName, ...) \
    const size_t testName ## Ret[] = { __VA_ARGS__ }; \
    const int testName[] =  

#define USE_TEST(testName) { .name = #testName, .path = testName, .path_size = sizeof(testName) / sizeof(testName[0]), .ret = testName ## Ret, .ret_size = sizeof(testName ## Ret) / sizeof(testName ## Ret[0]) }


DEFINE_TEST(noLoop, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0) {
    #include "no_loop.txt"
};

DEFINE_TEST(simpleLoop, 8, 1, 0) {
    #include "simple_loop.txt"
};

// aka:                      21               9  31      13       2
DEFINE_TEST(complexLoop, 50, 49, 20, 19, 18, 17, 10, 30, 29, 28, 27, 1, 0) {
    #include "complex_loop.txt"
};

// aka:                      8
// aka:                     29                   2
// aka:                     36                  16
DEFINE_TEST(largeNodes, 48, 47, 28, 27, 26, 25, 24, 1, 0) {
    #include "large_nodes.txt"
};

// aka:                                          14
DEFINE_TEST(fromSim, 48, 47, 46, 45, 44, 43, 42, 41, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0) {
    #include "from_sim.txt"
};




class TrackerTest : public UnitTest
{
public:
	TrackerTest();
	virtual ~TrackerTest();

	bool run_tests(void);

private:

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
        float x, y, z;
    };

    float get_distance(float x1, float y1, float z1, float x2, float y2, float z2);

    // Returns the distance of the specified point to a line in the specified test
    float distance_to_line(float x, float y, float z, const test_t *test, size_t line_end_index);

    // Returns true if the specified position represents a progress on the shortest path home.
    // If not, or if the position is not close to the path, the function returns false.
    bool detect_progress(float new_x, float new_y, float new_z, return_state_t &state, bool &did_divert);

    // Returns true if the return state has reached the home position
    bool detect_completion(return_state_t &state);

    // Initializes the return path supervision from any given position
    return_state_t init_return_state(float x, float y, float z, const test_t *test);

    // Makes the tracker return along the shortest path home and tests whether it chooses the right path.
    // Returns false if the test fails and fills msg with an appropriate error message.
    bool try_return_supervised(Tracker &tracker, const test_t *test, char *msg);

    // Makes the tracker return along the shortest path home. Use this if there is no well known return path. 
    // Returns false if the test fails and fills msg with an appropriate error message.
    bool try_return_unsupervised(Tracker &tracker, const test_t *test, float home_x, float home_y, float home_z, char *msg);

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

    // An array of flight paths and their correct return paths
    // We can do various tests on each of these.
    static const test_t test_cases[];
};

const TrackerTest::test_t TrackerTest::test_cases[] = {
    USE_TEST(noLoop),
    USE_TEST(simpleLoop),
    USE_TEST(complexLoop),
    USE_TEST(largeNodes),
    USE_TEST(fromSim)
};


TrackerTest::TrackerTest() {
}

TrackerTest::~TrackerTest() {
}


float TrackerTest::get_distance(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}


float TrackerTest::distance_to_line(float x, float y, float z, const test_t *test, size_t line_end_index) {
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

    return sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z) / sqrt(base_x * base_x + base_y * base_y + base_z * base_z);
}


bool TrackerTest::detect_progress(float new_x, float new_y, float new_z, return_state_t &state, bool &did_divert) {
    did_divert = true;
    bool did_advance = false;
    size_t target_index = state.target_index;

    do {
        float target_x = state.test->path[state.test->ret[target_index] * 3];
        float target_y = state.test->path[state.test->ret[target_index] * 3 + 1];
        float target_z = state.test->path[state.test->ret[target_index] * 3 + 2];

        float old_dist_to_target = get_distance(target_x, target_y, target_z, state.x, state.y, state.z);
        float new_dist_to_target = get_distance(target_x, target_y, target_z, new_x, new_y, new_z);

        if (new_dist_to_target < old_dist_to_target) {
            did_advance = true;
            if (distance_to_line(new_x, new_y, new_z, state.test, target_index) < Tracker::ACCURACY) {
                state.target_index = target_index;
                state.x = new_x;
                state.y = new_y;
                state.z = new_z;
                did_divert = false;
                break;
            }
        }
    } while (++target_index < state.test->ret_size);

    return did_advance;
}


bool TrackerTest::detect_completion(return_state_t &state) {
    float home_x = state.test->path[0];
    float home_y = state.test->path[1];
    float home_z = state.test->path[2];

    float dist_to_home = get_distance(state.x, state.y, state.z, home_x, home_y, home_z);
    return dist_to_home < Tracker::ACCURACY;
}


TrackerTest::return_state_t TrackerTest::init_return_state(float x, float y, float z, const test_t *test) {
    return {
        .test = test,
        .target_index = 0,
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


bool TrackerTest::try_return_supervised(Tracker &tracker, const test_t *test, char *msg) {
    // Return along the shortest path while checking if it's what we expect
    
    Tracker::pos_handle_t pos;
    float x, y, z;
    inner_assert(tracker.get_current_pos(pos, x, y, z), "tracker did not report current position");
    return_state_t state = init_return_state(x, y, z, test);

    for (;;) {

        if (!tracker.get_path_to_home(pos, x, y, z))
            break;

        inner_assert(tracker.is_same_pos(pos, pos), "position equality test failed");

        bool did_divert;
        inner_assert(detect_progress(x, y, z, state, did_divert), "no progress detected from (%f, %f, %f) to (%f, %f, %f), target index is %zu", state.x, state.y, state.z, x, y, z, state.target_index);
        inner_assert(!did_divert, "vehicle diverted from expected return path");

        // From the current position, prefetch the return path
        Tracker::pos_handle_t inner_pos;
        float inner_x, inner_y, inner_z;
        inner_assert(tracker.get_current_pos(inner_pos, inner_x, inner_y, inner_z), "tracker did not report current position");
        return_state_t inner_state = init_return_state(inner_x, inner_y, inner_z, test);

        while (tracker.get_path_to_home(inner_pos, inner_x, inner_y, inner_z)) {
            inner_assert(detect_progress(inner_x, inner_y, inner_z, inner_state, did_divert), "no progress detected in look-ahead return path from (%f, %f, %f) to (%f, %f, %f), target index is %zu", inner_state.x, inner_state.y, inner_state.z, inner_x, inner_y, inner_z, inner_state.target_index);
            inner_assert(!did_divert, "look-ahead return path diverted from expected return path");
        }
        
        inner_assert(detect_completion(inner_state), "look-ahead return path did not reach home, ended at (%f, %f, %f), target index is %zu", inner_state.x, inner_state.y, inner_state.z, inner_state.target_index);

        // Follow the return path by one position
        tracker.update(x, y, z);
    }

    inner_assert(detect_completion(state), "vehicle did not reach home, currently at (%f, %f, %f), home is (%d, %d, %d)", state.x, state.y, state.z, test->path[0], test->path[1], test->path[2]);

    return true;
}


bool TrackerTest::try_return_unsupervised(Tracker &tracker, const test_t *test, float home_x, float home_y, float home_z, char *msg) {
    int steps = 0;
    float x, y, z;
    Tracker::pos_handle_t pos;
    
    // As long as we're not home, return along the proposed path
    inner_assert(tracker.get_current_pos(pos, x, y, z), "tracker did not report current position");
    while (tracker.get_path_to_home(pos, x, y, z)) {
        tracker.update(x, y, z);
        inner_assert(steps++ < 256, "return-to-home did take too many steps"); // make sure the loop terminates
        TRACKER_DBG("return from %d, %d, %d, home is %.3f, %.3f, %.3f", (int)x, (int)y, (int)z, home_x, home_y, home_z);
    }

    // Check if we're actually home
    inner_assert(get_distance(x, y, z, home_x, home_y, home_z) <= Tracker::ACCURACY, "the vehicle didn't return home");

    return true;
}


bool TrackerTest::fly_and_return_test(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        TRACKER_DBG("running fly-and-return on %s", test->name);
        
        // Simulate flight along the specified path
        for (size_t p = 0; p < test->path_size; p += 3) {
            TRACKER_DBG("push %zu", p / 3);
            tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
        }


#ifdef DEBUG_TRACKER
        tracker.dump_graph();
#endif
 
        // Return home
        char msg[1024];
        if (!try_return_supervised(tracker, test, msg)) {
            ut_assert(msg, false);
        }
    }

	return true;
}


bool TrackerTest::fly_and_leave_return_path_test(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        TRACKER_DBG("running fly-and-leave-return-path on %s", test->name);
        
        // Simulate flight along the specified path
        for (size_t p = 0; p < test->path_size; p += 3)
            tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);

        int x, y, z;

        // Follow half of the return path
        for (size_t r = 0; r < test->ret_size / 2; r++) {
            x = test->path[test->ret[r] * 3];
            y = test->path[test->ret[r] * 3 + 1];
            z = test->path[test->ret[r] * 3 + 2];
            tracker.update(x, y, z);
        }

        // Select a destination using a deterministic pseudorandom index
        size_t dest_index = (x + y + z + test->ret[test->ret_size / 2]) % (test->path_size / 3);
        int dest_x = test->path[dest_index * 3];
        int dest_y = test->path[dest_index * 3 + 1];
        int dest_z = test->path[dest_index * 3 + 2];

        // Fly to the specified index (todo: when the tracker's delta limit gets fixed, we can directly jump to the destination).
        while (x != dest_x || y != dest_y || z != dest_z) {
            x += std::min(15, std::max(-16, dest_x - x));
            y += std::min(15, std::max(-16, dest_y - y));
            z += std::min(15, std::max(-16, dest_z - z));
            tracker.update(x, y, z);
        }

#ifdef DEBUG_TRACKER
        tracker.dump_graph();
#endif

        // Return home
        char msg[1024];
        if (!try_return_unsupervised(tracker, test, test->path[0], test->path[1], test->path[2], msg)) {
            ut_assert(msg, false);
        }
    }

	return true;
}


bool TrackerTest::fly_and_change_home_test(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        TRACKER_DBG("running fly-and-change-home on %s", test->name);
        
        // Simulate flight along the specified path
        for (size_t p = 0; p < test->path_size; p += 3)
            tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);

        // Select a home using a deterministic pseudorandom index
        size_t dest_index = (test->path[0] + test->path[1] + test->path[2] + test->ret[test->ret_size / 2]) % (test->path_size / 3);
        int home_x = test->path[dest_index * 3];
        int home_y = test->path[dest_index * 3 + 1] + Tracker::ACCURACY / 2; // add a little bit of offset
        int home_z = test->path[dest_index * 3 + 2] + Tracker::ACCURACY / 2; // add a little bit of offset

        tracker.set_home(home_x, home_y, home_z);

#ifdef DEBUG_TRACKER
        tracker.dump_graph();
#endif

        // Return home
        char msg[1024];
        if (!try_return_unsupervised(tracker, test, home_x, home_y, home_z, msg)) {
            ut_assert(msg, false);
        }
    }

	return true;
}


bool TrackerTest::run_tests(void) {
	ut_run_test(fly_and_return_test);
	ut_run_test(fly_and_leave_return_path_test);
	ut_run_test(fly_and_change_home_test);

	return (_tests_failed == 0);
}

ut_declare_test(trackerTest, TrackerTest)
