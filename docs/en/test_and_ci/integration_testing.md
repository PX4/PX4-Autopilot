# Integration Testing using ROS

This topic explains how to run (and extend) PX4's ROS-based integration tests.

::: info
[MAVSDK Integration Testing](../test_and_ci/integration_testing_mavsdk.md) is preferred when writing new tests.
Use the ROS-based integration test framework for use cases that _require_ ROS (e.g. object avoidance).

All PX4 integraton tests are executed automatically by our [Continuous Integration](../test_and_ci/continous_integration.md) system.
:::

## Prerequisites:

- [jMAVSim Simulator](../sim_jmavsim/index.md)
- [Gazebo Classic Simulator](../sim_gazebo_classic/index.md)
- [ROS and MAVROS](../simulation/ros_interface.md)

## Execute Tests

To run the MAVROS tests:

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

For example:

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test
```

Tests can also be run with a GUI to see what's happening (by default the tests run "headless"):

```sh
./test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test gui:=true headless:=false
```

The **.test** files launch the corresponding Python tests defined in `integrationtests/python_src/px4_it/mavros/`

## Write a New MAVROS Test (Python)

This section explains how to write a new python test using ROS 1/MAVROS, test it, and add it to the PX4 test suite.

We recommend you review the existing tests as examples/inspiration ([integrationtests/python_src/px4_it/mavros/](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros)).
The official ROS documentation also contains information on how to use [unittest](http://wiki.ros.org/unittest) (on which this test suite is based).

To write a new test:

1. Create a new test script by copying the empty test skeleton below:

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

1. Run the new test only

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

1. Add new test node to a launch file

   - In `test/` create a new `<test_name>.test` ROS launch file.
   - Call the test file using one of the base scripts _rostest_px4_run.sh_ or _rostest_avoidance_run.sh_

1. (Optional) Create a new target in the Makefile

   - Open the Makefile
   - Search the _Testing_ section
   - Add a new target name and call the test

   For example:

   ```sh
   tests_<new_test_target_name>: rostest
   	@"$(SRC_DIR)"/test/rostest_px4_run.sh mavros_posix_tests_<new_test>.test
   ```

Run the tests as described above.
