
/**
 * Navigator unit tests. Run the tests as follows:
 *   nsh> navigator_tests
 *
 */

//#include <systemlib/err.h>

#include "tracker_test.h"

extern "C" __EXPORT int navigator_tests_main(int argc, char *argv[]);


int navigator_tests_main(int argc, char *argv[])
{
	return trackerTest() ? 0 : -1;
}
