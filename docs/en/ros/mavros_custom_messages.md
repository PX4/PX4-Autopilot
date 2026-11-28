# Sending a Custom Message from MAVROS to PX4

:::warning
This article has been tested against:

- **Ubuntu:** 20.04
- **ROS:** Noetic
- **PX4 Firmware:** v1.13

However these steps are fairly general and so it should work with other distros/versions with little to no modifications.
:::

<!-- Content reproduced with permission from @JoonmoAhn in https://github.com/JoonmoAhn/Sending-Custom-Message-from-MAVROS-to-PX4/issues/1 -->

## MAVROS Installation

Follow _Source Installation_ instructions from [mavlink/mavros](https://github.com/mavlink/mavros/blob/master/mavros/README.md) to install "ROS Kinetic".

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

1. Edit **mavros_plugins.xml** (in **workspace/src/mavros/mavros_extras**) and add the following lines:

   ```xml
   <class name="keyboard_command" type="mavros::extra_plugins::KeyboardCommandPlugin" base_class_type="mavros::plugin::PluginBase">
        <description>Accepts keyboard command.</description>
   </class>
   ```

1. Edit **CMakeLists.txt** (in **workspace/src/mavros/mavros_extras**) and add the following line in `add_library`.

   ```cmake
   add_library(
   ...
     src/plugins/keyboard_command.cpp
   )
   ```

1. Inside **common.xml** in (**workspace/src/mavlink/message_definitions/v1.0**), copy the following lines to add your MAVLink message:
   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

## PX4 Changes

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

1. Make your own uORB message file **key_command.msg** in (PX4-Autopilot/msg). For this example the "key_command.msg" has only the code:

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

1. Edit **mavlink_receiver.h** (in **PX4-Autopilot/src/modules/mavlink**)

   ```cpp
   ...
   #include <uORB/Publication.hpp>
   #include <uORB/topics/key_command.h>
   ...
   class MavlinkReceiver
   {
   ...
   private:
       void handle_message_key_command(mavlink_message_t *msg);
   ...
       uORB::Publication<key_command_s> _key_command_pub{ORB_ID(key_command)};
   }
   ```

1. Edit **mavlink_receiver.cpp** (in **PX4-Autopilot/src/modules/mavlink**).
   This is where PX4 receives the MAVLink message sent from ROS, and publishes it as a uORB topic.

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

       key_command_s key{};
       key.cmd = man.command;
       key.timestamp = hrt_absolute_time();
       _key_command_pub.publish(key);
   }
   ```

1. Make your own uORB topic subscriber just like any example subscriber module.
   For this example lets create the model in (/PX4-Autopilot/src/modules/key_receiver).
   In this directory, create three files **CMakeLists.txt**, **key_receiver.cpp**, **Kconfig**
   Each one looks like the following.

   -CMakeLists.txt

   ```cmake
   px4_add_module(
       MODULE modules__key_receiver
       MAIN key_receiver
       SRCS
           key_receiver.cpp
       DEPENDS
           px4_work_queue
       )
   ```

   -key_receiver.cpp

   ```cpp
   #include <px4_platform_common/log.h>
   #include <px4_platform_common/module.h>
   #include <px4_platform_common/px4_work_queue/WorkItem.hpp>
   #include <uORB/SubscriptionCallback.hpp>
   #include <uORB/topics/key_command.h>

   class KeyReceiver : public ModuleBase, public px4::WorkItem
   {
   public:
       static Descriptor desc;

       KeyReceiver() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

       static int task_spawn(int argc, char *argv[]);
       static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
       static int print_usage(const char *reason = nullptr);

       void request_stop() override
       {
           ModuleBase::request_stop();
           ScheduleNow(); // Run() is not scheduled while no key is pressed, so schedule it to exit
       }

   private:
       void Run() override;

       // Schedules Run() whenever key_command is published
       uORB::SubscriptionCallbackWorkItem _key_command_sub{this, ORB_ID(key_command)};
   };

   ModuleBase::Descriptor KeyReceiver::desc{task_spawn, custom_command, print_usage};

   void KeyReceiver::Run()
   {
       if (should_exit()) {
           _key_command_sub.unregisterCallback();
           exit_and_cleanup(desc);
           return;
       }

       key_command_s key;

       if (_key_command_sub.update(&key)) {
           PX4_INFO("Received Char : %c", key.cmd);
       }
   }

   int KeyReceiver::task_spawn(int argc, char *argv[])
   {
       KeyReceiver *instance = new KeyReceiver();

       if (instance == nullptr) {
           PX4_ERR("alloc failed");
           return PX4_ERROR;
       }

       desc.object.store(instance);
       desc.task_id = task_id_is_work_queue;

       if (!instance->_key_command_sub.registerCallback()) {
           PX4_ERR("callback registration failed");
           delete instance;
           desc.object.store(nullptr);
           desc.task_id = -1;
           return PX4_ERROR;
       }

       return PX4_OK;
   }

   int KeyReceiver::print_usage(const char *reason)
   {
       if (reason) {
           PX4_WARN("%s\n", reason);
       }

       PRINT_MODULE_USAGE_NAME("key_receiver", "template");
       PRINT_MODULE_USAGE_COMMAND("start");
       PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

       return 0;
   }

   extern "C" __EXPORT int key_receiver_main(int argc, char *argv[])
   {
       return ModuleBase::main(KeyReceiver::desc, argc, argv);
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

   Start it with `key_receiver start`.
   For a more detailed explanation see the topic [Writing your first application](../modules/hello_sky.md).

1. Lastly, add your module in the **default.px4board** file correspondent to your board in **PX4-Autopilot/boards/**.
   For example:
   -for the Pixhawk 4, add the following code in **PX4-Autopilot/boards/px4/fmu-v5/default.px4board**:
   -for the SITL, add the following code in **PX4-Autopilot/boards/px4/sitl/default.px4board**

   ```
    CONFIG_MODULES_KEY_RECEIVER=y
   ```

Now you are ready to build all your work!

## Building

### Build for ROS

1. In your workspace enter: `catkin build`.
1. Beforehand, you have to set your "px4.launch" in (/workspace/src/mavros/mavros/launch).
   Edit "px4.launch" as below.
   If you are using USB to connect your computer with Pixhawk, you have to set "fcu_url" as shown below.
   But, if you are using CP2102 to connect your computer with Pixhawk, you have to replace "ttyACM0" with "ttyUSB0".
   And if you are using the SITL to connect to your terminal, you have to replace "/dev/ttyACM0:57600" with "udp://:14540@127.0.0.1:14557".
   Modifying "gcs_url" is to connect your Pixhawk with UDP, because serial communication cannot accept MAVROS, and your nutshell connection simultaneously.

1. Write your IP address at "xxx.xx.xxx.xxx"
   ```xml
   ...
     <arg name="fcu_url" default="/dev/ttyACM0:57600" />
     <arg name="gcs_url" default="udp://:14550@xxx.xx.xxx.xxx:14557" />
   ...
   ```

### Build for PX4

1. Clean the previously built PX4-Autopilot directory. In the root of **PX4-Autopilot** directory:

   ```sh
   make clean
   ```

1. Build PX4-Autopilot and upload [in the normal way](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards).

   For example:
   - to build for Pixhawk 4/FMUv5 execute the following command in the root of the PX4-Autopilot directory:

   ```sh
   make px4_fmu-v5_default upload
   ```

   - to build for SITL execute the following command in the root of the PX4-Autopilot directory (using SIH simulation):

   ```sh
   make px4_sitl_sih sihsim_quadx
   ```

## Running the Code

Next test if the MAVROS message is sent to PX4.

### Running ROS

1. In a terminal enter
   ```sh
   roslaunch mavros px4.launch
   ```
1. In a second terminal run:
   ```sh
   rostopic pub -r 10 /mavros/keyboard_command/keyboard_sub std_msgs/Char 97
   ```
   This means, publish 97 ('a' in ASCII) to ROS topic "/mavros/keyboard_command/keyboard_sub" in message type "std_msgs/Char". "-r 10" means to publish continuously in "10Hz".

### Running PX4

1. Enter the Pixhawk nutshell through UDP.
   Replace xxx.xx.xxx.xxx with your IP.

   ```sh
   cd PX4-Autopilot/Tools
   ./mavlink_shell.py xxx.xx.xxx.xxx:14557 --baudrate 57600
   ```

1. After few seconds, press **Enter** a couple of times.
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
