# 单元测试

我们鼓励开发人员在开发的每个模块时编写单元测试，包括添加新功能，修复错误和重构。

或者，也可以直接从 bash 运行完整的单元测试：

1. Unit tests with [Google Test](https://github.com/google/googletest/blob/main/docs/primer.md) ("GTest") - tests that have minimal, internal-only dependencies
2. 使用GTest的功能性测试 - 依赖parameters和 uORB消息的测试
3. 软件在环(SITL)单元测试。 这些测试需要运行在完整的SITL环境中， 运行起来更慢，更难调试，所以建议尽可能使用GTest代替。

## 编写测试

**Tip**: In general, if you need access to advanced GTest utilities, data structures from the STL or need to link to `parameters` or `uorb` libraries you should use the functional tests instead.

创建新的单元测试步骤如下：

1. 单元测试分成三个部分：设置、运行、检查结果。 每个单元测试都应该测试一个特定行为或设置案例，如果测试失败，则很明显你的测试代码有错误。 请尽可能遵循这些标准。
2. Copy and rename the example unit test [AttitudeControlTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) to the directory the code to be tested is in.
3. Add the new file to the directory's `CMakeLists.txt`. It should look something like `px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`
4. 添加你想要的测试功能。 这包括了添加所需的头文件、新测试(每个测试都应该有单独的名称)，并加入相关逻辑，运行测试代码并验证其行为是否符合预期。
5. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/unit-MyNewUnit`.
也可以直接通过调试器中运行。

## 写一个GTest功能测试

当测试或测试的组件依赖参数、uORB 消息、或更高级的GTest功能的时候，应当使用GTest功能测试。
Additionally, functional tests can contain local usage of STL data structures (although be careful of platform differences between e.g. macOS and Linux).

创建一个新的功能测试步骤如下：

1. 一般来说（与单元测试类似）功能测试应分为三个部分：设置，运行，检查结果。
   每个单元测试都应该测试一个特定行为或设置案例，如果测试失败，则很明显你的测试代码有错误。
   请尽可能遵循这些标准。
2. Copy and rename the example functional test [ParameterTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/parameters/ParameterTest.cpp) to the directory the code to be tested is in.
3. 将ParameterTest 重命名为更符合你正在测试的代码功能。
4. Add the new file to the directory's `CMakeLists.txt`.
   It should look something like `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
5. 添加你想要的测试功能。
   这包括了，添加特定的头文件、新测试（每个测试都应该使用不同的命名），并设置相关逻辑，运行测试代码并验证是否符合预期。
6. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/functional-MyNewFunctional`.
It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=\<regex\>](https://github.com/google/googletest/blob/main/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## 写一个软件在环（SITL）单元测试

当需要所有的飞行控制组件：驱动、时间或者更多时，应该SITL单元测试。
这些测试运行较慢(每个模块至少1秒+)，同时难以测试，所以仅在必要时使用它们。

创建一个新的SITL单元测试步骤如下：

1. Examine the sample [Unittest-class](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/unit_test.h).

2. Create a new .cpp file within [tests](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/tests) with name **test\_[description].cpp**.

3. Within **test\_[description].cpp** include the base unittest-class `<unit_test.h>` and all files required to write a test for the new feature.

4. Within **test\_[description].cpp** create a class `[Description]Test` that inherits from `UnitTest`.

5. Within `[Description]Test` class declare the public method `virtual bool run_tests()`.

6. Within `[Description]Test` class declare all private methods required to test the feature in question (`test1()`, `test2()`,...).

7. Within **test\_[description].cpp** implement the `run_tests()` method where each test[1,2,...] will be run.

8. Within **test\_[description].cpp**, implement the various tests.

9. At the bottom within **test\_[description].cpp** declare the test.

   ```cpp
   ut_declare_test_c(test_[description], [Description]Test)
   ```

   下面是一个模板：

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

10. Within [tests_main.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/tests_main.h) define the new test:

   ```cpp
   extern int test_[description](int argc, char *argv[]);
   ```

11. Within [tests_main.c](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/tests_main.c) add description name, test function and option:

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

   或

   ```sh
   pxh> tests jig
   ```

   If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use.

12. Add the test `test_[description].cpp` to the [CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/CMakeLists.txt).

## 在本地计算机上进行测试

Run the complete list of GTest Unit Tests, GTest Functional Tests and SITL Unit Tests right from bash:

```sh
make tests
```

The individual GTest test binaries are in the `build/px4_sitl_test/` directory, and can be run directly in most IDEs' debugger.

使用以下命令对ctest名称使用正则表达式对要运行的测试子集进行筛选：

```sh
make tests TESTFILTER=<regex filter expression>
```

例如：

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test
