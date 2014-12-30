# PX4 Nodes

This directory contains several small ROS nodes which are intended to run alongside the PX4 multi-platform nodes in
ROS. They act as a bridge between the PX4 specific topics and the ROS topics.

## Joystick Input

You will need to install the ros joystick packages
See http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

### Arch Linux
```sh
yaourt -Sy ros-indigo-joystick-drivers --noconfirm
```
check joystick
```sh
ls /dev/input/
ls -l /dev/input/js0
```
(replace 0 by the number you find with the first command)

make sure the joystick is accessible by the `input` group and that your user is in the `input` group
