# Raspberry Pi - ROS 安装

这是本文指导如何在树莓派2上安装 ROS-indigo ，部署成与 Pixhawk 协同的一台地面站计算机的。

## 系统必备组件

- 具有显示器、键盘或配置 ssh 连接的工作树莓派
- 本指南假定您的 RPi 上安装了 Raspbian "JESSIE"。 If not: [install it](https://www.raspberrypi.org/downloads/raspbian/) or [upgrade](http://raspberrypi.stackexchange.com/questions/27858/upgrade-to-raspbian-jessie) your Raspbian Wheezy to Jessie.

## 安装

Follow [this guide](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi) for the actual installation of ROS Indigo. 注意：安装 "ROS-Comm" 变体。 桌面变体太臃肿了。

### 安装程序包时出错

If you want to download packages (e.g. `sudo apt-get install ros-indigo-ros-tutorials`), you might get an error saying: "unable to locate package ros-indigo-ros-tutorials".

如果是这样，请按以下步骤操作：转到您的 catkin 工作区（例如 ~/ros_catkin_ws）并更改包的名称。

```sh
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_tutorials --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall
```

接下来，使用 wstool 更新您的工作区。

```sh
$ wstool merge -t src indigo-custom_ros.rosinstall

$ wstool update -t src
```

下一步（仍在工作区文件夹中），source 并创建文件。

```sh
$ source /opt/ros/indigo/setup.bash

$ source devel/setup.bash

$ catkin_make
```
