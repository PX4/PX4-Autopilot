# Integration Testing for the PX4 ROS 2 Interface Library

This topic outlines the integration tests for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).



## CI Testing

При відкритті запиту на PX4, CI запускає тест з інтеграції до бібліотеки.

## Running Tests Locally

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
