# Fuzz Tests

Fuzz tests are a generalised form of [unit test](../test_and_ci/unit_tests.md) that run code against a large number of random inputs.
This helps to ensure that the code is robust against any input, not just those expected by the developer.

They can be written like unit tests with possible assertions (`EXPECT_EQ`, etc), and have a set of input parameters.
The fuzzer then tries to find inputs that cause the code to crash (with [Address Sanitizer](https://clang.llvm.org/docs/AddressSanitizer.html) enabled automatically) or trigger an assertion.

The tests are run as part of normal unit tests, and in a more comprehensive fuzzing mode test.
For more information see [Running Fuzz Tests](#running-fuzz-tests) below.

## Writing a Fuzz Test

To write a fuzz test:

1. Start by writing a "normal" [functional test](../test_and_ci/unit_tests.md#functional-test).
2. Make sure the file name contains `fuzz` (lower case).
  For example `my_driver_fuzz_tests.cpp`.
3. Add one or more fuzz tests to the file.
  예:

   ```cpp
   #include <gtest/gtest.h>
   #include <fuzztest/fuzztest.h>
   
   void myDriverNeverCrashes(const std::string& s) {
      MyDriver driver;
      driver.handleInput(s);
   }
   FUZZ_TEST(MyDriverFuzzTests, myDriverNeverCrashes);
   ```

The file can also contain normal tests.
For more information, see https://github.com/google/fuzztest.

A complete example in the PX4 repository can be found in the [septentrio driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/gnss/septentrio/septentrio_fuzz_tests.cpp).

## Running Fuzz Tests

Fuzz tests can be run in two modes:

- As part of normal unit tests with `make tests`.
  This will only create a small number of inputs and not use coverage information.
- In fuzzing mode: this runs a single fuzz test with coverage information over a longer period of time (either fixed or indefinitely).
  The fuzzer will try to find inputs that cover all reachable code paths.
  It requires compilation with Clang and can be run with the following commands:

  ```sh
  rm -rf build/px4_sitl_tests
  export CC=clang
  export CXX=clang++
  make tests TESTFILTER=__no_tests__
  cd build/px4_sitl_tests
  ./functional-<my-test> --fuzz=<test-name>
  ```

## Seeds

Depending on the code complexity, it might be hard for the fuzzer to find inputs that pass certain conditions.
For this it is possible to provide one or more seeds, which the fuzzer will use as first inputs.
[The google documentation](https://github.com/google/fuzztest/blob/main/doc/fuzz-test-macro.md#initial-seeds-initial-seeds) contains more details.

You can also use the `FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION` macro for conditional code compilation, for example to exclude CRC checks.

More information about efficient fuzzing can be found on [this page](https://chromium.googlesource.com/chromium/src/+/main/testing/libfuzzer/efficient_fuzzing.md).

## CI

Fuzz tests are run as part of the normal unit tests in CI for each pull request.
In addition, the fuzz tests are run daily for 15 minutes on the main branch.
