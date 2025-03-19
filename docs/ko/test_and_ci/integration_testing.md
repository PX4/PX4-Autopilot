# ROS 통합 테스트

PX4의 ROS 기반 통합 테스트 방법을 설명합니다.

:::info
[MAVSDK Integration Testing](../test_and_ci/integration_testing_mavsdk.md) is preferred when writing new tests.
Use the ROS-based integration test framework for use cases that _require_ ROS (e.g. object avoidance).

All PX4 integraton tests are executed automatically by our [Continuous Integration](../test_and_ci/continous_integration.md) system.
:::

## 전제 조건

- [jMAVSim Simulator](../sim_jmavsim/index.md)
- [Gazebo Classic Simulator](../sim_gazebo_classic/index.md)
- [ROS and MAVROS](../simulation/ros_interface.md)

## 테스트 실행

MAVROS 테스트를 실행합니다.

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

예:

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test
```

테스트를 GUI로 실행하여 진행 상황을 쉽게 확인할 수도 있습니다(기본적으로 테스트는 "헤드리스"로 실행됨).

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test gui:=true headless:=false
```

The **.test** files launch the corresponding Python tests defined in `integrationtests/python_src/px4_it/mavros/`

## 신규 MAVROS 테스트 작성(Python)

This section explains how to write a new python test using ROS 1/MAVROS, test it, and add it to the PX4 test suite.

We recommend you review the existing tests as examples/inspiration ([integrationtests/python_src/px4_it/mavros/](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros)).
The official ROS documentation also contains information on how to use [unittest](http://wiki.ros.org/unittest) (on which this test suite is based).

새 테스트를 작성하려면:

1. 아래의 빈 테스트 스켈레톤을 복사하여 새 테스트 스크립트를 작성합니다.

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

2. 새 테스트만 실행합니다.

   - Start the simulator:

      ```sh
      cd <PX4-Autopilot_clone>
      source Tools/simulation/gazebo/setup_gazebo.bash
      roslaunch launch/mavros_posix_sitl.launch
      ```

   - Run test (in a new shell):

      ```sh
      cd <PX4-Autopilot_clone>
      source Tools/simulation/gazebo/setup_gazebo.bash
      rosrun px4 mavros_new_test.py
      ```

3. 시작 파일에 새 테스트 노드 추가

   - In `test/` create a new `<test_name>.test` ROS launch file.
   - Call the test file using one of the base scripts _rostest_px4_run.sh_ or _rostest_avoidance_run.sh_

4. (Optional) Create a new target in the Makefile

   - Open the Makefile
   - Search the _Testing_ section
   - Add a new target name and call the test

   예:

   ```sh
   tests_<new_test_target_name>: rostest
   	@"$(SRC_DIR)"/test/rostest_px4_run.sh mavros_posix_tests_<new_test>.test
   ```

위에서 설명한 대로 테스트를 실행합니다.
