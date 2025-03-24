# MAVROS _Offboard_ control example (Python)

This tutorial shows the basics of _OFFBOARD_ control with MAVROS Python, using an Iris quadcopter simulated in [Gazebo Classic](../sim_gazebo_classic/index.md).
Він надає покрокові інструкції, що демонструють як почати розробку програм для керування засобом та виконання коду в симуляції.

У кінці посібника ви повинні побачити таку ж поведінку, як і в нижченаведеному відео, повільний зліт на висоту 2 метри.

:::warning
_OFFBOARD_ control is dangerous.
Якщо ви керуєте реальним транспортним засобом, то обов'язково майте можливість отримати назад ручне керування на випадок, якщо щось піде не так.
:::

:::tip
This example uses Python.
Other examples in Python can be found here: [integrationtests/python_src/px4_it/mavros](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros).
:::

<a id="offb_video"></a>

<video width="100%" autoplay="true" controls="true">
 <source src="../../assets/simulation/gazebo_classic/gazebo_offboard.webm" type="video/webm">
</video>

## Створення пакету ROS

1. Open the terminal and go to `~/catkin_ws/src` directory

 ```sh
 roscd  # Should cd into ~/catkin_ws/devel
 cd ..
 cd src
 ```

2. In the `~/catkin_ws/src` directory create a new package named `offboard_py` (in this case) with the `rospy` dependency:

 ```sh
 catkin_create_pkg offboard_py rospy
 ```

3. Build the new package in the `~/catkin_ws/` directory:

 ```sh
 cd .. # Assuming previous directory to be ~/catkin_ws/src
 catkin build
 source devel/setup.bash
 ```

4. Тепер ви можете мати можливість перейти до пакета, використовуючи:

 ```sh
 ```

5. To store your Python files, create a new folder called `/scripts` on the package:

 ```sh
 mkdir scripts
 cd scripts
 ```

## Код

Після створення пакету ROS та директорії скриптів, ви готові до запуску вашого Python скрипту.
Inside the scripts folder create the `offb_node.py` file and give it executable permissions:

```sh
touch offb_node.py
chmod +x offb_node.py
```

After that, open `offb_node.py` file and paste the following code:

```py
"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()

```

## Пояснення коду

The `mavros_msgs` package contains all of the custom messages required to operate services and topics provided by the MAVROS package.
All services and topics as well as their corresponding message types are documented in the [mavros wiki](http://wiki.ros.org/mavros).

```py
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
```

Ми створюємо простий виклик, який буде зберігати поточний стан автопілота.
This will allow us to check connection, arming and _OFFBOARD_ flags.:

```py
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg
```

Ми створюємо екземпляр видавця для публікації командної локальної позиції та відповідних клієнтів для запиту на arming та зміни режиму.
Зверніть увагу, що для вашої власної системи, префікс "mavros" може відрізнятися, так як це буде залежати від імені, даного вузлу в файлі запуску.

```py
state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

rospy.wait_for_service("/mavros/cmd/arming")
arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

rospy.wait_for_service("/mavros/set_mode")
set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
```

PX4 has a timeout of 500ms between two _OFFBOARD_ commands.
If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering _OFFBOARD_ mode.
This is why the publishing rate **must** be faster than 2 Hz to also account for possible latencies.
This is also the same reason why it is **recommended to enter _OFFBOARD_ mode from _Position_ mode**, this way if the vehicle drops out of _OFFBOARD_ mode it will stop in its tracks and hover.

Тут ми відповідно встановлюємо швидкість публікації:

```py
# Setpoint publishing MUST be faster than 2Hz
rate = rospy.Rate(20)
```

Перш ніж щось публікувати, ми чекаємо встановлення зв'язку між MAVROS і автопілотом.
Цей цикл має закінчитись, щойно буде отримано повідомлення про hearbeat.

```py
# Wait for Flight Controller connection
while(not rospy.is_shutdown() and not current_state.connected):
    rate.sleep()
```

Попри те, що PX4 працює в координатній площині NED, MAVROS переводить ці координати до ENU стандарту та навпаки.
This is why we set `z` to positive 2:

```py
pose = PoseStamped()

pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2
```

Before entering _OFFBOARD_ mode, you must have already started streaming setpoints.
В іншому випадку перемикач режиму буде відхилено.
Below, `100` was chosen as an arbitrary amount.

```py
# Send a few setpoints before starting
for i in range(100):
    if(rospy.is_shutdown()):
        break

    local_pos_pub.publish(pose)
    rate.sleep()
```

We prepare the message request used to set the custom mode to `OFFBOARD`.
A list of [supported modes](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) is available for reference.

```py
offb_set_mode = SetModeRequest()
offb_set_mode.custom_mode = 'OFFBOARD'
```

Решта коду є значною мірою поясненням.
We attempt to switch to _Offboard_ mode, after which we arm the quad to allow it to fly.
Ми визначаємо паузу виклику сервісів у 5 секунд, щоб не перевантажити автопілот запитами.
В тому ж циклі ми продовжуємо надсилати запитану позицію за частотою, яка раніше визначена.

```py
arm_cmd = CommandBoolRequest()
arm_cmd.value = True

last_req = rospy.Time.now()

while(not rospy.is_shutdown()):
    if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        if(set_mode_client.call(offb_set_mode).mode_sent == True):
            rospy.loginfo("OFFBOARD enabled")

        last_req = rospy.Time.now()
    else:
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")

            last_req = rospy.Time.now()

    local_pos_pub.publish(pose)

    rate.sleep()
```

:::tip
This code has been simplified to the bare minimum for illustration purposes.
У великих системах часто корисно створити новий потік, який буде відповідати за періодичну публікацію заданих значень.
:::

## Створення ROS launch файлу

In your `offboard_py` package, create another folder inside the `~/catkin_ws/src/offboard_py/src` directory named `launch`.
Саме тут будуть зберігатися файли запуску пакету.
After that, create your first launch file, in this case we will call it `start_offb.launch`.

```sh
roscd offboard_py
mkdir launch
cd launch
touch start_offb.launch
```

For the `start_offb.launch` copy the following code:

```xml
<?xml version="1.0"?>
<launch>
	<!-- Include the MAVROS node with SITL and Gazebo -->
	<include file="$(find px4)/launch/mavros_posix_sitl.launch">
	</include>

	<!-- Our node to control the drone -->
	<node pkg="offboard_py" type="offb_node.py" name="offb_node_py" required="true" output="screen" />
</launch>
```

As you can see, the `mavros_posix_sitl.launch` file is included.
This file is responsible for launching MAVROS, the PX4 SITL, the Gazebo Classic Environment and for spawning a vehicle in a given world (for further information see the file [here](https://github.com/PX4/PX4-Autopilot/blob/main/launch/mavros_posix_sitl.launch)).

:::tip
The `mavros_posix_sitl.launch` file takes several arguments that can be set according to your preferences such as the vehicle to spawn or the Gazebo Classic world (refer to [here](https://github.com/PX4/PX4-Autopilot/blob/main/launch/mavros_posix_sitl.launch)) for a complete list).

You can override the default value of these arguments defined in `mavros_posix_sitl.launch` by declaring them inside the _include_ tags.
As an example, if you wanted to spawn the vehicle in the `warehouse.world`, you would write the following:

```xml
<!-- Include the MAVROS node with SITL and Gazebo -->
<include file="$(find px4)/launch/mavros_posix_sitl.launch">
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/warehouse.world"/>
</include>
```

:::

## Запуск скрипту

Якщо все зроблено правильно, ви повинні тепер мати можливість запустити і протестувати свій скрипт.

В терміналі запустить:

```sh
roslaunch offboard_py start_offb.launch
```

Тепер ви повинні побачити ініціацію прошивки PX4 і виконання застосунку в Gazebo Classic.
After the _OFFBOARD_ mode is set and the vehicle is armed, the behavior shown in the [video](#offb_video) should be observed.

:::warning
It is possible that when running the script an error appears saying:

> Resource not found: px4
> ROS path [0] = ...
> ...

Це означає, що PX4 SITL не включено в path.
To solve this add these lines at the end of the `.bashrc` file:

```sh
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
```

Тепер у терміналі, перейдіть до домашнього каталогу і виконайте наступну команду, щоб застосувати зміни вище до поточного терміналу:

```sh
source .bashrc
```

Після цього кроку, кожного разу, коли ви відкриваєте нове вікно терміналу, вас не повинна турбувати ця помилка.
If it appears again, a simple `source .bashrc` should fix it.
This solution was obtained from this [issue](https://github.com/mzahana/px4_fast_planner/issues/4) thread, where you can get more information about the problem.
:::
