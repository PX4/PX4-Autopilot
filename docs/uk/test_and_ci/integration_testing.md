# Тестування інтеграції з використанням ROS

У цій темі пояснюється, як запускати (і розширювати) інтеграційні тести PX4 на основі ROS.

:::info
[MAVSDK Integration Testing](../test_and_ci/integration_testing_mavsdk.md) is preferred when writing new tests.
Use the ROS-based integration test framework for use cases that _require_ ROS (e.g. object avoidance).

All PX4 integraton tests are executed automatically by our [Continuous Integration](../test_and_ci/continous_integration.md) system.
:::

## Попередня підготовка:

- [jMAVSim Simulator](../sim_jmavsim/index.md)
- [Gazebo Classic Simulator](../sim_gazebo_classic/index.md)
- [ROS and MAVROS](../simulation/ros_interface.md)

## Виконати тести

Щоб запустити тести MAVROS:

```sh
source <catkin_ws>/devel/setup.bash
cd <PX4-Autopilot_clone>
make px4_sitl_default sitl_gazebo
make <test_target>
```

`test_target` is a makefile targets from the set: _tests_mission_, _tests_mission_coverage_, _tests_offboard_ and _tests_avoidance_.

Test can also be executed directly by running the test scripts, located under `test/`:

```sh
source <catkin_ws>/devel/setup.bash
cd <PX4-Autopilot_clone>
make px4_sitl_default sitl_gazebo
./test/<test_bash_script> <test_launch_file>
```

Наприклад:

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test
```

Тести також можна запускати за допомогою графічного інтерфейсу користувача, щоб побачити, що відбувається (за замовчуванням тести виконуються без голови):

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test gui:=true headless:=false
```

The **.test** files launch the corresponding Python tests defined in `integrationtests/python_src/px4_it/mavros/`

## Напишіть новий MAVROS-тест (Python)

Цей розділ пояснює, як написати новий python тест з використанням ROS 1/MAVROS, протестувати його та додати до набору тестів PX4.

We recommend you review the existing tests as examples/inspiration ([integrationtests/python_src/px4_it/mavros/](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros)).
The official ROS documentation also contains information on how to use [unittest](http://wiki.ros.org/unittest) (on which this test suite is based).

Щоб написати новий тест:

1. Створити новий тестовий скрипт, копіюючи порожній тестовий каркас нижче:

   ```python
   #!/usr/bin/env python
   # [... LICENSE ...]

   #
   # @author Example Author <author@example.com>
   #
   PKG = 'px4'

   import unittest
   import rospy
   import rosbag

   from sensor_msgs.msg import NavSatFix

   class MavrosNewTest(unittest.TestCase):
   	"""
   	Test description
   	"""

   	def setUp(self):
   		rospy.init_node('test_node', anonymous=True)
   		rospy.wait_for_service('mavros/cmd/arming', 30)

   		rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
   		self.rate = rospy.Rate(10) # 10hz
   		self.has_global_pos = False

   	def tearDown(self):
   		pass

   	#
   	# General callback functions used in tests
   	#
   	def global_position_callback(self, data):
   		self.has_global_pos = True

   	def test_method(self):
   		"""Test method description"""

   		# FIXME: hack to wait for simulation to be ready
   		while not self.has_global_pos:
   			self.rate.sleep()

   		# TODO: execute test

   if __name__ == '__main__':
   	import rostest
   	rostest.rosrun(PKG, 'mavros_new_test', MavrosNewTest)
   ```

2. Запустити лише новий тест

   - Запустити симулятор

      ```sh
      cd <PX4-Autopilot_clone>
      source Tools/simulation/gazebo/setup_gazebo.bash
      roslaunch launch/mavros_posix_sitl.launch
      ```

   - Запустити тест (в новій оболонці):

      ```sh
      cd <PX4-Autopilot_clone>
      source Tools/simulation/gazebo/setup_gazebo.bash
      rosrun px4 mavros_new_test.py
      ```

3. Додати новий тестовий вузол до файлу запуску

   - In `test/` create a new `<test_name>.test` ROS launch file.
   - Call the test file using one of the base scripts _rostest_px4_run.sh_ or _rostest_avoidance_run.sh_

4. (Необов'язково) Створити нову ціль в Makefile

   - Відкрийте Makefile
   - Search the _Testing_ section
   - Додати нову назву цілі та викликати тест

   Наприклад:

   ```sh
   tests_<new_test_target_name>: rostest
   	@"$(SRC_DIR)"/test/rostest_px4_run.sh mavros_posix_tests_<new_test>.test
   ```

Запустити тести, як описані вище.
