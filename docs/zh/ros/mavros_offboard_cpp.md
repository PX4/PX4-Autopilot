# MAVROS _Offboard_ control example (C++)

This tutorial shows the basics of _Offboard_ control with MAVROS, using an Iris quadcopter simulated in Gazebo Classic/SITL.
在教程结束时，你应该看到与下面的视频相同的行为, 即缓慢起飞到2米的高度。

:::warning
_Offboard_ control is dangerous.
如果你是在一个真正的无人机平台上进行试验，请保证你已经设置了切换回手动的开关来防止紧急情况的发生。
:::

:::tip
This example uses C++.
A very similar example for Python can be found in [ROS/MAVROS Offboard Example (Python)](../ros/mavros_offboard_python.md) (also see the examples in [integrationtests/python_src/px4_it/mavros](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros)).
:::

<video width="100%" autoplay="true" controls="true">
  <source src="../../assets/simulation/gazebo_classic/gazebo_offboard.webm" type="video/webm">
</video>

## 代码

Create the `offb_node.cpp` file in your ROS package (by also adding it to your `CMakeList.txt` so it is compiled), and paste the following inside it:

```cpp
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

```

## 代码解释

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```

The `mavros_msgs` package contains all of the custom messages required to operate services and topics provided by the MAVROS package.
All services and topics as well as their corresponding message types are documented in the [mavros wiki](http://wiki.ros.org/mavros).

```cpp
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

我们创建了一个简单的回调函数来储存飞控当前的状态。
This will allow us to check connection, arming and _Offboard_ flags.

```cpp
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

我们构建了一个发布者来发布本地位置指令并请求客户端进行加解锁状态及控制模式的切换。
请注意，对于您自己的系统，"mavros" 前缀可能不同，取决于节点启动文件中指定的名称。

```cpp
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```

PX4 has a timeout of 500ms between two _Offboard_ commands.
If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering _Offboard_ mode.
This is why the publishing rate **must** be faster than 2 Hz to also account for possible latencies.
This is also the same reason why it is recommended to enter _Offboard_ mode from _Position_ mode, this way if the vehicle drops out of _Offboard_ mode it will stop in its tracks and hover.

```cpp
// wait for FCU connection
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

在发布任何消息之前，我们需要等待飞控和MAVROS建立连接。
在收到心跳包之后，代码便会跳出这个循环。

```cpp
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

尽管PX4在航空航天常用的NED坐标系下操控飞机，但MAVROS将自动将该坐标系切换至常规的ENU坐标系下，反之亦然。
This is why we set `z` to positive 2.

```cpp
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
  local_pos_pub.publish(pose);
  ros::spinOnce();
  rate.sleep();
}
```

Before entering _Offboard_ mode, you must have already started streaming setpoints.
否则，模式切换将被拒绝。 Here, `100` was chosen as an arbitrary amount.

```cpp
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```

We set the custom mode to `OFFBOARD`.
A list of [supported modes](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) is available for reference.

```cpp
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
  if( current_state.mode != "OFFBOARD" &&
    (ros::Time::now() - last_request > ros::Duration(5.0))){
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      ROS_INFO("Offboard enabled");
    }
    last_request = ros::Time::now();
  } else {
    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
      }
      last_request = ros::Time::now();
    }
  }

  local_pos_pub.publish(pose);

  ros::spinOnce();
  rate.sleep();
}
```

该代码的其余部分完全是自解释性的。
We attempt to switch to _Offboard_ mode, after which we arm the quad to allow it to fly.
我们每隔五秒去调用一次该服务，避免飞控被大量的请求阻塞。
在同一个循环中，我们按照指定的频率持续发送期望点设定值信息给飞控。

:::tip
This code has been simplified to the bare minimum for illustration purposes.
在一个复杂的系统中，通常需要创建新的进程来负责周期性的发送位置期望值给飞控。
:::
