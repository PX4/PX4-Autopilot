# Unit Tests

Developers are encouraged to write unit tests during all parts of development, including adding new features, fixing bugs, and refactoring.

PX4 provides several methods for writing unit tests:

1. Unit tests with [Google Test](https://github.com/google/googletest/blob/main/docs/primer.md) ("GTest") - tests that have minimal, internal-only dependencies
1. Functional tests with GTest - tests that depend on parameters and uORB messages
1. SITL unit tests. This is for tests that need to run in full SITL. These tests are much slower to run and harder to debug, so it is recommended to use GTest instead when possible.

## Writing a GTest Unit Test

**Tip**: In general, if you need access to advanced GTest utilities, data structures from the STL or need to link to `parameters` or `uorb` libraries you should use the functional tests instead.

The steps to create new unit tests are as follows:

1. Unit tests should be arranged in three sections: setup, run, check results. Each test should test one very specific behavior or setup case, so if a test fails it is obvious what is wrong. Please try to follow these standards when possible.
1. Copy and rename the example unit test [AttitudeControlTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) to the directory the code to be tested is in.
1. Add the new file to the directory's `CMakeLists.txt`. It should look something like `px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`
1. Add the desired test functionality. This will mean including the header files required for your specific tests, adding new tests (each with an individual name) and putting the logic for the setup, running the code to be tested and verifying that it behaves as expected.
1. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/unit-MyNewUnit`.
It can be run directly in a debugger.

## Writing a GTest Functional Test

GTest functional tests should be used when the test or the components being tested depend on parameters, uORB messages and/or advanced GTest functionality.
Additionally, functional tests can contain local usage of STL data structures (although be careful of platform differences between e.g. macOS and Linux).

The steps to creating new functional tests are as follows:

1. In general (and similar to unit tests), functional tests should be arranged in three sections: setup, run, check results.
   Each test should test one very specific behavior or setup case, so if a test fails it is obvious what is wrong.
   Please try to follow these standards when possible.
1. Copy and rename the example functional test [ParameterTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/parameters/ParameterTest.cpp) to the directory the code to be tested is in.
1. Rename the class from ParameterTest to something better representing the code being testing
1. Add the new file to the directory's `CMakeLists.txt`.
   It should look something like `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
1. Add the desired test functionality.
   This will mean including the header files required for your specific tests, adding new tests (each with an individual name) and putting the logic for the test setup, running the code to be tested and verifying that it behaves as expected.
1. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/functional-MyNewFunctional`.
It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=\<regex\>](https://github.com/google/googletest/blob/main/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## Writing a SITL Unit Test

SITL unit tests should be used when you specifically need all of the flight controller components - drivers, time, and more.
These tests are slower to run (1s+ for each new module), and harder to debug, so in general they should only be used when necessary.

The steps to create new SITL unit tests are as follows:

1. Examine the sample [Unittest-class](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/unit_test.h).
1. Create a new .cpp file within [tests](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/tests) with name **test\_[description].cpp**.
1. Within **test\_[description].cpp** include the base unittest-class `<unit_test.h>` and all files required to write a test for the new feature.
1. Within **test\_[description].cpp** create a class `[Description]Test` that inherits from `UnitTest`.
1. Within `[Description]Test` class declare the public method `virtual bool run_tests()`.
1. Within `[Description]Test` class declare all private methods required to test the feature in question (`test1()`, `test2()`,...).
1. Within **test\_[description].cpp** implement the `run_tests()` method where each test[1,2,...] will be run.
1. Within **test\_[description].cpp**, implement the various tests.
1. At the bottom within **test\_[description].cpp** declare the test.

   ```cpp
   ut_declare_test_c(test_[description], [Description]Test)
   ```

   Here is a template:

   ```cpp
   #include <unit_test.h>
   #include "[new feature].h"
   ...

   class [Description]Test : public UnitTest
   {
   public:
       virtual bool run_tests();

   private:
       bool test1();
       bool test2();
       ...
   };

   bool [Description]Test::run_tests()
   {
       ut_run_test(test1)
       ut_run_test(test2)
       ...

       return (_tests_failed == 0);
   }

   bool [Description]Test::test1()
   {
       ut_[name of one of the unit test functions](...
       ut_[name of one of the unit test functions](...
       ...

       return true;
   }

   bool [Description]Test::test2()
   {
       ut_[name of one of the unit test functions](...
       ut_[name of one of the unit test functions](...
       ...

       return true;
   }
   ...

   ut_declare_test_c(test_[description], [Description]Test)
   ```

   Note that `ut_[name of one of the unit test functions]` corresponds to one of the unittest functions defined within [unit_test.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/unit_test.h).

1. Within [tests_main.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/tests_main.h) define the new test:

   ```cpp
   extern int test_[description](int argc, char *argv[]);
   ```

1. Within [tests_main.c](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/tests_main.c) add description name, test function and option:

   ```cpp
   ...
   } tests[] = {
       {...
       {"[description]", test_[description], OPTION},
       ...
   }
   ```

   `OPTION` can be `OPT_NOALLTEST`,`OPT_NOJIGTEST` or `0` and is considered if within px4 shell one of the two commands are called:

   ```sh
   pxh> tests all
   ```

   or

   ```sh
   pxh> tests jig
   ```

   If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use.

1. Add the test `test_[description].cpp` to the [CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/CMakeLists.txt).

## Testing on the local machine

Run the complete list of GTest Unit Tests, GTest Functional Tests and SITL Unit Tests right from bash:

```sh
make tests
```

The individual GTest test binaries are in the `build/px4_sitl_test/` directory, and can be run directly in most IDEs' debugger.

Filter to run only a subset of tests using a regular expression for the ctest name with this command:

```sh
make tests TESTFILTER=<regex filter expression>
```

For example:

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test
