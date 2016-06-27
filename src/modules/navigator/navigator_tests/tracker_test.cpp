
/**
 * Flight path tracker tests
 *
 */

#include "tracker_test.h"

#include "../tracker.h"
#include <unit_test/unit_test.h>
#include <cstring> // std::strcat


#define make_test(testName) { .name = #testName, .path = testName, .path_size = sizeof(testName) / sizeof(testName[0]), .ret = testName ## Ret, .ret_size = sizeof(testName ## Ret) / sizeof(testName ## Ret[0]) }

const int noLoop[] = {
    #include "no_loop.txt"
};
const size_t noLoopRet[] = { 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

const int simpleLoop[] = {
    #include "simple_loop.txt"
};
const size_t simpleLoopRet[] = { 9, 8, 1, 0 };

class TrackerTest : public UnitTest
{
public:
	TrackerTest();
	virtual ~TrackerTest();

	bool run_tests(void);

private:
	bool graphTest();

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
        make_test(noLoop),
        make_test(simpleLoop)
};


TrackerTest::TrackerTest() {
}

TrackerTest::~TrackerTest() {
}

bool TrackerTest::graphTest(void) {
    for (size_t t = 0; t < sizeof(test_cases) / sizeof(test_cases[0]); t++) {
        Tracker tracker;

        const test_t *test = test_cases + t;
        char msg[50] = "tracker failed on ";
        std::strcat(msg, test->name);
        
        // Simulate flight along the specified path
        for (size_t p = 0; p < test->path_size / 3; p++)
            tracker.update(test->path[p * 3], test->path[p * 3 + 1], test->path[p * 3 + 2]);

tracker.dump_graph();
        int x[test->ret_size];
        int y[test->ret_size];
        int z[test->ret_size];

        // Return along the shortest path while checking if it's what we expect
        for (size_t r = 0; r < test->ret_size; r++) {
            int fetched = tracker.get_path_to_home(x, y, z, test->ret_size - r);

            ut_assert(msg, fetched != 0); // If we're not home yet, the tracker must give us at least one position
            ut_assert(msg, fetched + r <= test->ret_size); // Ensure that the return path is not too long

            for (size_t k = 0; k < fetched; k++) {
                ut_compare(msg, x[k], test->path[test->ret[r + k] * 3]);
                ut_compare(msg, y[k], test->path[test->ret[r + k] * 3 + 1]);
                ut_compare(msg, z[k], test->path[test->ret[r + k] * 3 + 2]);
            }

            // Follow the return path by one position
            tracker.update(*x, *y, *z);
        }

        int fetched = tracker.get_path_to_home(x, y, z, 3);
        ut_assert(msg, fetched == 0); // Now we should be home
    }

	return true;
}

bool TrackerTest::run_tests(void) {
	ut_run_test(graphTest);

	return (_tests_failed == 0);
}

ut_declare_test(trackerTest, TrackerTest)
