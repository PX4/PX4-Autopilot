# Модульні Тести

Розробникам рекомендується писати модульні тести на всіх етапах розробки, включаючи додавання нових функцій, виправлення помилок і рефакторинг

PX4 надає декілька методів для написання юніт тестів:

1. Unit tests with [Google Test](https://github.com/google/googletest/blob/main/docs/primer.md) ("GTest") - tests that have minimal, internal-only dependencies
2. Функціональні тести з GTest - тести, які залежать від параметрів та uORB повідомлень
3. Модульні тести SITL. Це і є тести, які повинні запускатися в повному SITL. Ці тести виконуються набагато повільніше та важче налагодити, тому, якщо можливо, замість них рекомендується використовувати GTest.

## Написання GTest Unit Test

**Tip**: In general, if you need access to advanced GTest utilities, data structures from the STL or need to link to `parameters` or `uorb` libraries you should use the functional tests instead.

Кроки для створення нових функціональних тестів такі:

1. Модульні тести мають бути організовані в три секції: налаштування, запуск, перевірка результатів. Кожен тест повинен перевіряти одну дуже конкретну поведінку або випадок налаштування, тому, якщо тест провалиться, стане очевидним, що не так. Будь ласка, намагайтеся дотримуватися цих стандартів, коли це можливо.
2. Copy and rename the example unit test [AttitudeControlTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) to the directory the code to be tested is in.
3. Add the new file to the directory's `CMakeLists.txt`. It should look something like `px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`
4. Додайте бажану функцію тестування. Це означатиме включення файлів заголовків, необхідних для ваших конкретних тестів, додавання нових тестів (кожен з індивідуальною назвою) і розміщення логіки для налаштування, запуск коду для перевірки та перевірка його поведінки, як очікувалося.
5. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/unit-MyNewUnit`.
Він може бути запущений безпосередньо в налагоджувачі.

## Написання GTest Functional Test

Функціональні тести GTest слід використовувати, коли тест або компоненти, що тестуються, залежать від параметрів, повідомлень uORB та/або розширеної функціональності GTest.
Крім того, функціональні тести можуть містити локальне використання структур даних STL (хоча і будьте обережні відмінності платформ між такими як macOS і Linux).

Кроки для створення нових функціональних тестів такі:

1. Загалом (і подібно до модульних тестів), функціональні тести мають бути організовані за трьома розділами: налаштування, запуск, перевірка результатів.
   Кожен тест повинен перевіряти одну дуже конкретну поведінку або випадок налаштування, тому, якщо тест провалиться, стане очевидним, що не так.
   Будь ласка, намагайтеся дотримуватися цих стандартів, коли це можливо.
2. Copy and rename the example functional test [ParameterTest](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/parameters/ParameterTest.cpp) to the directory the code to be tested is in.
3. Перейменуйте клас з ParameterTest на те, що краще представляє код, що тестується
4. Add the new file to the directory's `CMakeLists.txt`.
   It should look something like `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
5. Додайте бажану функцію тестування.
   Це означатиме включення файлів заголовків, необхідних для ваших конкретних тестів, додавання нових тестів (кожен з індивідуальною назвою) і розміщення логіки для налаштування тесту, запуск коду, який потрібно перевірити, і перевірку його поведінки, як очікувалося.
6. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/functional-MyNewFunctional`.
It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=\<regex\>](https://github.com/google/googletest/blob/main/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## Написання SITL Unit Test

Модульні тести SITL слід використовувати, коли вам конкретно потрібні всі компоненти контролера польоту – водії, час тощо.
Ці тести виконуються повільніше (1 с+ для кожного нового модуля) і їх важче налагодити, тому їх слід використовувати лише за необхідності.

Кроки для створення нових модульних тестів SITL такі:

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

   Тут є шаблон:

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

   або

   ```sh
   pxh> tests jig
   ```

   If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use.

12. Add the test `test_[description].cpp` to the [CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/systemcmds/tests/CMakeLists.txt).

## Тестування на локальній машині

Запустіть повний список модульних тестів GTest, функціональних тестів GTest і модульних тестів SITL прямо з bash:

```sh
make tests
```

The individual GTest test binaries are in the `build/px4_sitl_test/` directory, and can be run directly in most IDEs' debugger.

Фільтр, щоб запустити лише підмножину тестів, використовуючи регулярний вираз для імені ctest за допомогою цієї команди:

```sh
make tests TESTFILTER=<regex filter expression>
```

Наприклад:

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test
