# MAVROS에서 PX4로 사용자 정의 메시지 전송

:::warning
This article has been tested against:

- **Ubuntu:** 20.04
- **ROS:** Noetic
- **PX4 Firmware:** v1.13

그러나 이러한 단계는 상당히 일반적이므로 수정이 거의 없이 다른 배포판/버전에서 작동합니다.
:::

<!-- Content reproduced with permission from @JoonmoAhn in https://github.com/JoonmoAhn/Sending-Custom-Message-from-MAVROS-to-PX4/issues/1 -->

## MAVROS 설치

Follow _Source Installation_ instructions from [mavlink/mavros](https://github.com/mavlink/mavros/blob/master/mavros/index.md) to install "ROS Kinetic".

## MAVROS

1. We start by creating a new MAVROS plugin, in this example named **keyboard_command.cpp** (in **workspace/src/mavros/mavros_extras/src/plugins**) by using the code below:

   The code subscribes a 'char' message from ROS topic `/mavros/keyboard_command/keyboard_sub` and sends it as a MAVLink message.

   ```c
    #include <mavros/mavros_plugin.h>
    #include <pluginlib/class_list_macros.h>
    #include <iostream>
    #include <std_msgs/Char.h>

    namespace mavros {
    namespace extra_plugins{

    class KeyboardCommandPlugin : public plugin::PluginBase {
    public:
        KeyboardCommandPlugin() : PluginBase(),
            nh("~keyboard_command")

       { };

        void initialize(UAS &uas_)
        {
            PluginBase::initialize(uas_);
            keyboard_sub = nh.subscribe("keyboard_sub", 10, &KeyboardCommandPlugin::keyboard_cb, this);
        };

        Subscriptions get_subscriptions()
        {
            return {/* RX disabled */ };
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber keyboard_sub;

       void keyboard_cb(const std_msgs::Char::ConstPtr &req)
        {
            std::cout << "Got Char : " << req->data <<  std::endl;
            mavlink::common::msg::KEY_COMMAND kc {};
            kc.command = req->data;
            UAS_FCU(m_uas)->send_message_ignore_drop(kc);
        }
    };
    }   // namespace extra_plugins
    }   // namespace mavros

   PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::KeyboardCommandPlugin, mavros::plugin::PluginBase)
   ```

2. Edit **mavros_plugins.xml** (in **workspace/src/mavros/mavros_extras**) and add the following lines:
   ```xml
   <class name="keyboard_command" type="mavros::extra_plugins::KeyboardCommandPlugin" base_class_type="mavros::plugin::PluginBase">
        <description>Accepts keyboard command.</description>
   </class>
   ```

3. Edit **CMakeLists.txt** (in **workspace/src/mavros/mavros_extras**) and add the following line in `add_library`.
   ```cmake
   add_library( 
   ...
     src/plugins/keyboard_command.cpp 
   )
   ```

4. Inside **common.xml** in (**workspace/src/mavlink/message_definitions/v1.0**), copy the following lines to add your MAVLink message:
   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

## PX4 수정사항

1. Inside **common.xml** (in **PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0**), add your MAVLink message as following (same procedure as for MAVROS section above):

   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

   :::warning
   Make sure that the **common.xml** files in the following directories are exactly the same:

   - `PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0`
   - `workspace/src/mavlink/message_definitions/v1.0`
      are exactly the same.

:::

2. Make your own uORB message file **key_command.msg** in (PX4-Autopilot/msg). 이 예에서 "key_command.msg"에는 다음 코드만 있습니다.

   ```
   uint64 timestamp # time since system start (microseconds)
   char cmd
   ```

   Then, in **CMakeLists.txt** (in **PX4-Autopilot/msg**), include:

   ```cmake
   set(
   ...
        key_command.msg
        )
   ```

3. Edit **mavlink_receiver.h** (in **PX4-Autopilot/src/modules/mavlink**)

   ```cpp
   ...
   #include <uORB/topics/key_command.h>
   ...
   class MavlinkReceiver
   {
   ...
   private:
       void handle_message_key_command(mavlink_message_t *msg);
   ...
       orb_advert_t _key_command_pub{nullptr};
   }
   ```

4. Edit **mavlink_receiver.cpp** (in **PX4-Autopilot/src/modules/mavlink**).
   여기에서 PX4는 ROS에서 보낸 MAVLink 메시지를 수신하고, 이를 uORB 주제로 게시합니다.
   ```cpp
   ...
   void MavlinkReceiver::handle_message(mavlink_message_t *msg)
   {
   ...
    case MAVLINK_MSG_ID_KEY_COMMAND:
           handle_message_key_command(msg);
           break;
   ...
   }
   ...
   void
   MavlinkReceiver::handle_message_key_command(mavlink_message_t *msg)
   {
       mavlink_key_command_t man;
       mavlink_msg_key_command_decode(msg, &man);

   struct key_command_s key = {};

       key.timestamp = hrt_absolute_time();
       key.cmd = man.command;

       if (_key_command_pub == nullptr) {
           _key_command_pub = orb_advertise(ORB_ID(key_command), &key);

       } else {
           orb_publish(ORB_ID(key_command), _key_command_pub, &key);
       }
   }
   ```

5. 예제 구독자 모듈처럼 자신만의 uORB 주제 구독자를 생성합니다.
   이 예에서는 (/PX4-Autopilot/src/modules/key_receiver)에서 모델을 생성할 수 있습니다.
   In this directory, create three files **CMakeLists.txt**, **key_receiver.cpp**, **Kconfig**
   Each one looks like the following.

   -CMakeLists.txt

   ```cmake
   px4_add_module(
       MODULE modules__key_receiver
       MAIN key_receiver
       STACK_MAIN 2500
       STACK_MAX 4000
       SRCS
           key_receiver.cpp
       DEPENDS

       )
   ```

   -key_receiver.cpp

   ```
   #include <px4_platform_common/px4_config.h>
   #include <px4_platform_common/tasks.h>
   #include <px4_platform_common/posix.h>
   #include <unistd.h>
   #include <stdio.h>
   #include <poll.h>
   #include <string.h>
   #include <math.h>

   #include <uORB/uORB.h>
   #include <uORB/topics/key_command.h>

   extern "C" __EXPORT int key_receiver_main(int argc, char **argv);

   int key_receiver_main(int argc, char **argv)
   {
       int key_sub_fd = orb_subscribe(ORB_ID(key_command));
       orb_set_interval(key_sub_fd, 200); // limit the update rate to 200ms

       px4_pollfd_struct_t fds[] = {
           { .fd = key_sub_fd,   .events = POLLIN },
       };

       int error_counter = 0;

       for (int i = 0; i < 10; i++)
       {
           int poll_ret = px4_poll(fds, 1, 1000);

           if (poll_ret == 0)
           {
               PX4_ERR("Got no data within a second");
           }

           else if (poll_ret < 0)
           {
               if (error_counter < 10 || error_counter % 50 == 0)
               {
                   PX4_ERR("ERROR return value from poll(): %d", poll_ret);
               }

               error_counter++;
           }

           else
           {
               if (fds[0].revents & POLLIN)
               {
                   struct key_command_s input;
                   orb_copy(ORB_ID(key_command), key_sub_fd, &input);
                   PX4_INFO("Received Char : %c", input.cmd);
                }
           }
       }
       return 0;
   }
   ```

   -Kconfig

   ```
    menuconfig MODULES_KEY_RECEIVER
    bool "key_receiver"
    default n
    ---help---
    	Enable support for key_receiver

   ```

   For a more detailed explanation see the topic [Writing your first application](../modules/hello_sky.md).

6. Lastly, add your module in the **default.px4board** file correspondent to your board in **PX4-Autopilot/boards/**.
   For example:
   -for the Pixhawk 4, add the following code in **PX4-Autopilot/boards/px4/fmu-v5/default.px4board**:
   -for the SITL, add the following code in **PX4-Autopilot/boards/px4/sitl/default.px4board**

   ```
    CONFIG_MODULES_KEY_RECEIVER=y
   ```

MODULES
...

## Building

### Build for ROS

1. In your workspace enter: `catkin build`.

2. Beforehand, you have to set your "px4.launch" in (/workspace/src/mavros/mavros/launch).
   Edit "px4.launch" as below.
   If you are using USB to connect your computer with Pixhawk, you have to set "fcu_url" as shown below.
   But, if you are using CP2102 to connect your computer with Pixhawk, you have to replace "ttyACM0" with "ttyUSB0".
   And if you are using the SITL to connect to your terminal, you have to replace "/dev/ttyACM0:57600" with "udp://:14540@127.0.0.1:14557".
   Modifying "gcs_url" is to connect your Pixhawk with UDP, because serial communication cannot accept MAVROS, and your nutshell connection simultaneously.

3. Write your IP address at "xxx.xx.xxx.xxx"
   ```xml
   ...
     <arg name="fcu_url" default="/dev/ttyACM0:57600" />
     <arg name="gcs_url" default="udp://:14550@xxx.xx.xxx.xxx:14557" />
   ...
   ```

### ROS 빌드

1. Clean the previously built PX4-Autopilot directory. In the root of **PX4-Autopilot** directory:
   ```sh
   make clean
   ```

2. Build PX4-Autopilot and upload [in the normal way](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards).

   예:

   - to build for Pixhawk 4/FMUv5 execute the following command in the root of the PX4-Autopilot directory:

   ```sh
   make px4_fmu-v5_default upload
   ```

   - to build for SITL execute the following command in the root of the PX4-Autopilot directory (using jmavsim simulation):

   ```sh
   make px4_sitl jmavsim
   ```

## 빌드

이제 작업을 빌드할 준비가 되었습니다!

### PX4 빌드

1. In a terminal enter
   ```sh
   roslaunch mavros px4.launch
   ```
2. In a second terminal run:
   ```sh
   rostopic pub -r 10 /mavros/keyboard_command/keyboard_sub std_msgs/Char 97
   ```
   This means, publish 97 ('a' in ASCII) to ROS topic "/mavros/keyboard_command/keyboard_sub" in message type "std_msgs/Char". "-r 10" means to publish continuously in "10Hz".

### ROS 실행

1. Enter the Pixhawk nutshell through UDP.
   Replace xxx.xx.xxx.xxx with your IP.
   ```sh
   cd PX4-Autopilot/Tools
   ./mavlink_shell.py xxx.xx.xxx.xxx:14557 --baudrate 57600
   ```

2. After few seconds, press **Enter** a couple of times.
   You should see a prompt in the terminal as below:
   ```sh
   nsh>
   nsh>
   ```
   Type "key_receiver", to run your subscriber module.
   ```
   nsh> key_receiver
   ```

Check if it successfully receives `a` from your ROS topic.
