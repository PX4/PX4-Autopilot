# 단위 테스트

개발자는 기능 추가, 버그 수정, 리팩토링 등 전 영역에 걸쳐 단위 테스트를 하는 것이 바람직합니다.

PX4에서는 단위 테스트 작성에 필요한 몇가지 수단을 제공합니다:

1. Unit tests with [Google Test](https://github.com/google/googletest/blob/main/docs/primer.md) ("GTest") - tests that have minimal, internal-only dependencies
2. GTest로의 기능 시험 - 매개변수와 uORB 메세지에 따른 시험
3. SITL 단위 테스트. 완전한 SITL 실행에 필요한 테스트입니다. 이 테스트는 실행하기에 매우 느리거나 디버깅하기 어려운 부분입니다. 따라서 가능하면 GTest를 활용하시는게 좋습니다.

## GTest 단위 테스트 작성

**Tip**: In general, if you need access to advanced GTest utilities, data structures from the STL or need to link to `parameters` or `uorb` libraries you should use the functional tests instead.

새 단위 테스트의 작성 절차는 다음과 같습니다:

1. 단위 테스트는 설치, 실행, 결과 검사 세 부분으로 정리해야 합니다. 각 테스트에서는 매우 극히 일부의 동작을 시험하거나 설정 조건을 시험하기 때문에, 테스트에 실패했을 경우 어떤 부분에서 문제가 있는지 명백하게 드러납니다. 가능하면 이 표준을 따라주십시오.
2. Copy and rename the example unit test [AttitudeControlTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) to the directory the code to be tested is in.
3. Add the new file to the directory's `CMakeLists.txt`. It should look something like `px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`
4. 원하는 시험 기능을 추가하십시오. 특정 테스트를 수행하려면 헤더 파일 추가가 필요하며, 새 테스트 추가(제각각의 이름을 지님), 설정 로직 배치, 시험할 코드 실행, 결과 검증을 기대대로 수행합니다.
5. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/unit-MyNewUnit`.
디버거에서 바로 실행할 수 있습니다.

## GTest 기능 테스트 작성

GTest 기능 시험은 매개변수, uORB 메세지, 고급 GTest 기능에 따라 테스트할 테스트 단위 또는 구성 요소가 있을 때 활용해야합니다.
게다가, 기능 테스트 과정에서 자체 STL 데이터 구조를 사용할 수 있습니다(플랫폼간 차이에 유의해야 함. 예: maxOS, Linux).

새 기능 테스트의 작성 절차는 다음과 같습니다:

1. 보통 (그리고 단위 테스트와 유사한  상황에서), 기능 테스트는 구성, 실행, 결과 검사 세부분으로 정리해야합니다.
   각 테스트에서는 매우 극히 일부의 동작을 시험하거나 설정 조건을 시험하기 때문에, 테스트에 실패했을 경우 어떤 부분에서 문제가 있는지 명백하게 드러납니다.
   가능하면 이 표준을 따라주십시오.
2. Copy and rename the example functional test [ParameterTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/parameters/ParameterTest.cpp) to the directory the code to be tested is in.
3. ParameterTest의 클래스 이름을 시험할 코드를 더 잘 구분할 수 있는 이름으로 변경하십시오.
4. Add the new file to the directory's `CMakeLists.txt`.
   It should look something like `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
5. 원하는 시험 기능을 추가하십시오.
   특정 테스트를 수행하려면 헤더 파일 추가가 필요하며, 새 테스트 추가(제각각의 이름을 지님), 설정 로직 배치, 시험할 코드 실행, 결과 검증을 기대대로 수행합니다.
6. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/functional-MyNewFunctional`.
It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=\<regex\>](https://github.com/google/googletest/blob/main/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## SITL 단위 테스트 작성

특히 비행체 제어 장치의 모든 부분 - 드라이버, 시간, 등을 시험하려면 SITL 단위 테스트를 거쳐야합니다.
이 테스트는 실행이 느리며(새 모듈 별로 1초씩 추가), 디버깅도 어려워, 보통 필요할 때만 테스트를 활용합니다.

새 SITL 단위 테스트의 작성 절차는 다음과 같습니다:

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

   서식은 아래와 같습니다:

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

   또는

   ```sh
   pxh> tests jig
   ```

   If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use.

12. Add the test `test_[description].cpp` to the [CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/CMakeLists.txt).

## 로컬 머신에서의 테스트

GTest 단위 시험, GTest 기능 시험, SITL 단위 시험 전체를 Bash 쉘에서 실행하십시오.

```sh
make tests
```

The individual GTest test binaries are in the `build/px4_sitl_test/` directory, and can be run directly in most IDEs' debugger.

테스트 하위 집합만 따로 실행하려면 이 명령에서 ctest 명칭에 대해 정규 표현식을 적용하여 걸러내십시오:

```sh
make tests TESTFILTER=<regex filter expression>
```

예:

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test
