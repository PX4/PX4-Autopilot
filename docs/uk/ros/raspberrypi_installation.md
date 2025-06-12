# Raspberry Pi - встановлення ROS

Це посібник про те, як встановити ROS-indigo на Raspberry Pi 2 який виступає комп'ютером компаньйоном для Pixhawk.

## Вимоги

- Робочий Raspberry Pi з монітором, клавіатурою, або налаштованим SSH з'єднанням
- Цей посібник передбачає, що у вас є Raspbian "JESSIE", встановлений на вашому RPi. If not: [install it](https://www.raspberrypi.org/downloads/raspbian/) or [upgrade](http://raspberrypi.stackexchange.com/questions/27858/upgrade-to-raspbian-jessie) your Raspbian Wheezy to Jessie.

## Встановлення

Follow [this guide](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi) for the actual installation of ROS Indigo. Примітка: встановіть варіант "ROS-Comm". Варіант Desktop занадто важкий.

### Помилки при встановленні пакетів

If you want to download packages (e.g. `sudo apt-get install ros-indigo-ros-tutorials`), you might get an error saying: "unable to locate package ros-indigo-ros-tutorials".

Якщо так, продовжуйте наступним чином:
Перейдіть до свого catkin робочого простору (наприклад, ~/ros_catkin_ws) та змініть ім'я пакетів.

```sh
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_tutorials --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall
```

Далі, оновіть своє робоче середовище з wstool.

```sh
$ wstool merge -t src indigo-custom_ros.rosinstall

$ wstool update -t src
```

Далі (досі у вашому робочому середовищі).

```sh
$ source /opt/ros/indigo/setup.bash

$ source devel/setup.bash

$ catkin_make
```
