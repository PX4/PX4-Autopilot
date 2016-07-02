
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

    // Simulates a flight along the defined test path,
    // and then returns on the shortest path while verifying that
    // the tracker recommends the expected return path.  
	bool flyAndReturnTest();

    // Simulates a flight along the defined test path,
    // returns half of the shortest path, flies to some other pseudorandom (deterministic)
    // position in the graph, and then returns along the shortest path from there. 
    bool flyAndLeaveReturnPathTest();

    struct test_t {
        const char *name;
        const int *path;
        const size_t path_size;
        const size_t *ret;
        const size_t ret_size;
    };

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

bool TrackerTest::flyAndReturnTest(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        char msg[50] = "tracker failed to return on ";
        std::strcat(msg, test->name);
        TRACKER_DBG("running fly-and-return on %s", test->name);
        
        // Simulate flight along the specified path
        for (size_t p = 0; p < test->path_size; p += 3) {
            TRACKER_DBG("push %zu", p / 3);
            tracker.update(test->path[p], test->path[p + 1], test->path[p + 2]);
        }


#ifdef DEBUG_TRACKER
        tracker.dump_graph();
#endif
 

        // Return along the shortest path while checking if it's what we expect
        for (size_t r = 0; r < test->ret_size; r++) {
            Tracker::pos_handle_t pos;
            float x, y, z;

            ut_assert(msg, tracker.get_current_pos(pos, x, y, z));
            ut_assert(msg, tracker.is_same_pos(pos, pos));

            for (size_t k = 0; k < test->ret_size - r; k++) {
                ut_assert(msg, tracker.get_path_to_home(pos, x, y, z));
                ut_compare(msg, (int)x, test->path[test->ret[r + k] * 3]);
                ut_compare(msg, (int)y, test->path[test->ret[r + k] * 3 + 1]);
                ut_compare(msg, (int)z, test->path[test->ret[r + k] * 3 + 2]);
            }
            
            ut_assert(msg, !tracker.get_path_to_home(pos, x, y, z));

            // Follow the return path by one position
            tracker.update(test->path[test->ret[r] * 3], test->path[test->ret[r] * 3 + 1], test->path[test->ret[r] * 3 + 2]);
        }
    }

	return true;
}


bool TrackerTest::flyAndLeaveReturnPathTest(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        char msg[50] = "tracker failed when leaving return path on ";
        std::strcat(msg, test->name);
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

        // As long as we're not home, return along the proposed path
        int steps = 0;
        float fx = x, fy = y, fz = z;
        Tracker::pos_handle_t pos;
        ut_assert(msg, tracker.get_current_pos(pos, fx, fy, fz));
        while ((int)fx != test->path[0] || (int)fy != test->path[1] || (int)fz != test->path[2]) {
            TRACKER_DBG("return from %d, %d, %d, home is %d, %d, %d", (int)fx, (int)fy, (int)fz, test->path[0], test->path[1], test->path[2]);
            ut_assert(msg, tracker.get_path_to_home(pos, fx, fy, fz));
            tracker.update(fx, fy, fz);
            ut_assert(msg, steps++ < INT_MAX); // make sure the loop terminates
        }

        // Check if the tracker agrees that we're home
        ut_assert(msg, !tracker.get_path_to_home(pos, fx, fy, fz));
    }

	return true;
}

bool TrackerTest::run_tests(void) {
	ut_run_test(flyAndReturnTest);
	ut_run_test(flyAndLeaveReturnPathTest);

	return (_tests_failed == 0);
}

ut_declare_test(trackerTest, TrackerTest)
