# Інтерфейсна бібліотека PX4 ROS 2

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Експериментальні налаштування
На момент написання цієї статті, деякі частини бібліотеки інтерфейсу PX4 ROS 2 є експериментальними і, отже, можуть бути змінені.
:::

[PX4 ROS 2 Інтерфейс бібліотеки](https://github.com/Auterion/px4-ros2-interface-lib) є бібліотекою C+++, яка спрощує контроль і взаємодіє з PX4 з ROS 2.

Бібліотека надає два інтерфейси високого рівня для розробників:

1. [Control Interface](./px4_ros2_interface.md) дозволяє розробникам створювати та динамічно реєструвати режими, написані з використанням ROS 2.
   Бібліотека також надає класи для надсилання різних типів налаштувань, починаючи від багаторівневих навігаційних завдань на високому рівні аж до прямого контролю приводу.
2. [Навігаційний інтерфейс](./px4_ros2_navigation_interface.md) дозволяє надсилати позицію автомобіля з позиції PX4 з ROS 2 додатків, таких як система VIO.

<!--
## Overview
-->

## Встановлення в робочому просторі ROS 2

Щоб почати використовувати бібліотеку в існуючому робочому просторі ROS 2:

1. Make sure you have a working [ROS 2 setup](../ros2/user_guide.md), with [`px4_msgs`](https://github.com/PX4/px4_msgs) in the ROS 2 workspace.

2. Клонуйте репозиторій в робочий простір:

   ```sh
   cd $ros_workspace/src
   git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
   ```

   :::info
   Для забезпечення сумісності, використовуйте останні _main_ гілки для PX4, _px4_msgs_ та бібліотеки.
   Дивіться також [here](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4).

:::

3. Побудуйте робочий простір:

   ```sh
   cd ..
   colcon build
   source install/setup.bash
   ```

<!--
## How to Use the Library
-->

##

При відкритті запиту на PX4, CI запускає тест з інтеграції до бібліотеки.

Тести можуть також бути виконані локально з PX4:

```sh
./test/ros_test_runner.py
```

І щоб керувати лише одним випадком:

```sh
./test/ros_test_runner.py --verbose --case <case>
```

Ви можете скласти список доступних тестових кейсів з:

```sh
./test/ros_test_runner.py --list-cases
```
