# Польотні завдання

_Польотні завдання_ використовуються у [Режимах польоту](../concept/flight_modes.md) для забезпечення певної поведінки під час руху, наприклад "слідуй за мною" або пом'якшування польоту.

## Загальний огляд

Польотне завдання є класом в програмному каркасі польотних завдань похідним від базового класу [FlightTask](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/tasks/FlightTask/FlightTask.hpp). Його мета - генерувати задані значення для контролера з довільних вхідних даних, де кожне завдання реалізує бажану поведінку рухомого засобу для певного режиму.
Розробники як правило перевизначаються віртуальні методи `activate()` та `update()` викликаючи мінімальну реалізацію базового завдання та розширяючи її реалізацією бажаної поведінки.
Метод `activate()` викликається при перемиканні на завдання та дозволяє ініціалізувати його стан й м'яко перебрати на себе функціонал базуючись на заданих значеннях які попереднє завдання тільки передало.

`update()` викликається на кожній ітерації циклу під час виконання і містить реалізацію базової поведінки створення заданих значень.

За загальним правилом, завдання містяться у піддиректорії [PX4-Autopilot/src/modules/flight_mode_manager/tasks](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/flight_mode_manager/tasks) за назвою завдання, а назви файлів вихідного коду мають префікс "FlightTask".

:::info
Відео огляди із зібрань розробників PX4 [надано нижче](#video).
:::

## Створення польотного завдання

Нижченаведені інструкції можуть бути використані для створення завдання _MyTask_:

1. Створіть каталог для нової польотної задачі в [PX4-Autopilot/src/modules/flight_mode_manager/tasks](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/flight_mode_manager/tasks).
   За правилами директорія називається за завданням, тому ми назвемо її **/MyTask**.

   ```sh
   mkdir PX4-Autopilot/src/modules/flight_mode_manager/tasks/MyTask
   ```

2. Створіть порожні файли вихідного коду та налаштування _cmake_ для нового завдання у директорії _MyTask_ використовуючи префікс "FlightTask":
   - CMakeLists.txt
   - FlightTaskMyTask.hpp
   - FlightTaskMyTask.cpp

3. Оновіть **CMakeLists.txt** для нового завдання

   - Скопіюйте вміст **CMakeLists.txt** з іншого завдання, наприклад, [Orbit/CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/tasks/Orbit/CMakeLists.txt)

   - Оновіть відмітку про авторське право до поточного року

      ```cmake
      ############################################################################
      #
      #   Copyright (c) 2021 PX4 Development Team. All rights reserved.
      #
      ```

   - Модифікуйте код щоб він відповідав новому завданню, наприклад замініть `FlightTaskOrbit` на `FlightTaskMyTask`.
      Код буде виглядати приблизно так:

      ```cmake
      px4_add_library(FlightTaskMyTask
          FlightTaskMyTask.cpp
      )

      target_link_libraries(FlightTaskMyTask PUBLIC FlightTask)
      target_include_directories(FlightTaskMyTask PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
      ```

4. Оновіть файл заголовків (у цьому випадку **FlightTaskMyTask. pp**): Більшість завдань повторно реалізує віртуальні методи `activate()` і `update()`, в цьому прикладі ми також маємо приватну змінну.

   ```cpp
   #pragma once

   #include "FlightTask.hpp"

   class FlightTaskMyTask : public FlightTask
   {
   public:
     FlightTaskMyTask() = default;
     virtual ~FlightTaskMyTask() = default;

     bool update();
     bool activate(const trajectory_setpoint_s &last_setpoint) override;

   private:
     float _origin_z{0.f};
   };
   ```

5. Оновіть cpp файли відповідно.
   Цей приклад надає як просту реалізацію **FlightTaskMyTask.cpp**, яка просто показує, що методи завдань викликаються.

   ```cpp
   #include "FlightTaskMyTask.hpp"

   bool FlightTaskMyTask::activate(const trajectory_setpoint_s &last_setpoint)
   {
     bool ret = FlightTask::activate(last_setpoint);
     PX4_INFO("FlightTaskMyTask activate was called! ret: %d", ret); // report if activation was successful
     return ret;
   }

   bool FlightTaskMyTask::update()
   {
     PX4_INFO("FlightTaskMyTask update was called!"); // report update
     return true;
   }
   ```

6. Add the new task to the list of tasks to be built in [PX4-Autopilot/src/modules/flight_mode_manager/CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/CMakeLists.txt#L41).

   ```cmake
   ...
    list(APPEND flight_tasks_all
      Auto
      Descend
      ...
      ManualPositionSmoothVel
      Transition
      MyTask
    )
   ...
   ```

   ::: tip

   The task added above will be built on all boards, including those with constrained flash such as Pixhawk FMUv2.
   If your task is not indended for use on boards with constrained flash it should instead be added to the conditional block shown below (as shown).

   ```cmake
   ...
   if(NOT px4_constrained_flash_build)
      list(APPEND flight_tasks_all
        AutoFollowTarget
        Orbit
        MyTask
      )
    endif()
   ...
   ```


:::

7. Оновіть режим польоту, щоб переконатися, що завдання було викликано.
   Зазвичай для обрання певного польотного завдання використовується параметр.

   Наприклад, щоб активувати наше нове завдання `MyTask` в позиційному режимі мультикоптера:

   - Оновіть `MPC_POS_MODE` ([multicopter_position_mode_params.](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/multicopter_position_mode_params.c)), щоб додати варіант для вибору "MyTask", якщо параметр має раніше невикористане значення, наприклад 5:

      ```c
      ...
       * @value 0 Direct velocity
       * @value 3 Smoothed velocity
       * @value 4 Acceleration based
       * @value 5 My task
       * @group Multicopter Position Control
       */
      PARAM_DEFINE_INT32(MPC_POS_MODE, 5);
      ```

   - Додайте мітку case для нового варіанту в операторі switch для параметра в [FlightModeManager.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/FlightModeManager.cpp#L266-L285), щоб увімкнути завдання коли `_param_mpc_pos_mode` має відповідне значення.

      ```cpp
      ...
      // manual position control
      ...
      switch (_param_mpc_pos_mode.get()) {
        ...
        case 3:
           error = switchTask(FlightTaskIndex::ManualPositionSmoothVel);
           break;
        case 5: // Add case for new task: MyTask
           error = switchTask(FlightTaskIndex::MyTask);
           break;
      case 4:
      ....
      ...
      ```

## Перевірка нового польотного завдання

Щоб перевірити польотне завдання треба запустити рухомий засіб з увімкненим завданням.
Для прикладу вище це означає встановити параметр `MPC_POS_MODE` у 5, злетіти та перемикнути засіб у [Режим позиціювання](../flight_modes_mc/position.md).

:::info
Завдання, визначене вище, повинно перевірятися тільки на симуляторі.
Код не створює заданих значень, тому засіб не полетить.
:::

Зберіть симуляцію SITL (gazebo-classic)

```sh
make px4_sitl gazebo-classic
```

Відкрийте QGroundControl (якщо не відкрито, жодне повідомлення не буде надруковано).
В консолі, злетіть та перемкніться у режим позиціювання:

```sh
pxh> commander takeoff
pxh> commander mode posctl
```

Консоль постійно буде показувати: `INFO [FlightTaskMyTask] FlightTaskMyTask update was called!`.
Якщо ви хочете змінити режим польоту на інший, ви можете ввести команду для зміни режиму, наприклад `commander mode altctl`.

## Відео

Наступні відео дають огляд польотних завдань в PX4.
Перше охоплює стан програмного каркаса для польотних завдань в PX4 v1.9.
Друге - це оновлення, яке охоплює зміни в PX4 v1.11.

### Огляд архітектури польотних завдань PX4 (PX4 Developer Summit 2019)

Опис того, як працюють режими польоту у PX4 v1.9 (Dennis Mannhart, Matthias Grob).

<lite-youtube videoid="-dkQG8YLffc" title="PX4 Flight Task Architecture Overview"/>

<!-- datestamp:video:youtube:20190704:PX4 Flight Task Architecture Overview — PX4 Developer Summit 2019 -->

### Огляд управління мультикоптером від датчиків до двигунів (PX4 Developer Summit Virtual 2020)

<lite-youtube videoid="orvng_11ngQ" params="start=560" title="Overview of multicopter control from sensors to motors"/>

<!-- datestamp:video:youtube:20200720:Overview of multicopter control from sensors to motors — PX4 Developer Summit Virtual 2020 From 9min20sec - Section on flight tasks-->

Відповідний розділ цього відео з оновленою інформацією про польотні завдання в PX4 v11.1 на (9 мін. 20 сек.).
[Слайди знаходяться тут (PDF)](https://static.sched.com/hosted_files/px4developersummitvirtual2020/1b/PX4%20Developer%20Summit%202020%20-%20Overview%20of%20multicopter%20control%20from%20sensors%20to%20motors.pdf), відповідні слайди - 9 та 12.
