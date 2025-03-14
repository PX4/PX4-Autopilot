# Польоти з використанням систем захоплення руху (VICON, NOKOV, Optitrack)

:::warning
**WORK IN PROGRESS**

This topic shares significant overlap with [External Position Estimation (ROS)](../ros/external_position_estimation.md).
:::

Системи захоплення руху у приміщенні, такі як VICON, NOKOV та Optitrack, можуть бути використані для надання даних про положення та орієнтацію для оцінки стану транспортного засобу або можуть бути використані як основа для аналізу.
Дані з систем захоплення руху можуть бути використані для оновлення локальної оцінки положення PX4 відносно локального початку координат Курс (поворот) з системи захоплення руху також може бути опціонально інтегрований оцінювачем положення.

Pose (position and orientation) data from the motion capture system is sent to the autopilot over MAVLink, using the [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) message. Дивіться розділ нижче про системи координат для норм представлення даних. The [mavros](../ros/mavros_installation.md) ROS-Mavlink interface has a default plugin to send this message. Їх також можна надсилати, використовуючи чистий код на мовах програмування C/C++ та бібліотеки MAVLink.

## Архітектура обчислювальних систем

It is **highly recommended** that you send motion capture data via an **onboard** computer (e.g Raspberry Pi, ODroid, etc.) for reliable communications. Вбудований комп'ютер може бути підключений до комп'ютера руху за допомогою WiFi, що забезпечує надійне, високопропускне з'єднання.

Most standard telemetry links like 3DR/SiK radios are **not** suitable for high-bandwidth motion capture applications.

## Системи координат

У цьому розділі показано, як налаштувати систему з відповідними опорними системами. Існує різноманітні представлення, але ми використовуватимемо два з них: ENU і NED.

- ENU is a ground-fixed frame where **X** axis points East, **Y** points North and **Z** up. The robot/vehicle body frame is **X** towards the front, **Z** up and **Y** towards the left.
- NED has **X** towards North, **Y** East and **Z** down. The robot/vehicle body frame has **X** towards the front, **Z** down and **Y** accordingly.

На зображенні нижче показані системи координат. NED ліворуч, ENU праворуч:

![Reference frames](../../assets/lpe/ref_frames.png)

With the external heading estimation, however, magnetic North is ignored and faked with a vector corresponding to world _x_ axis (which can be placed freely at mocap calibration); yaw angle will be given respect to local _x_.

:::warning
When creating the rigid body in the motion capture software, remember to first align the robot with the world **X** axis otherwise yaw estimation will have an initial offset.
:::

## Оціночники стану

EKF2 рекомендується для систем з GPS (LPE застаріла, тому її більше не підтримується або не підтримується).
Q-Estimator рекомендується, якщо у вас немає GPS, оскільки він працює без магнітометра або барометра.

See [Switching State Estimators](../advanced/switching_state_estimators.md) for more information.

### EKF2

The ROS topic for motion cap `mocap_pose_estimate` for mocap systems and `vision_pose_estimate` for vision.
Check [mavros_extras](http://wiki.ros.org/mavros_extras) for further info.

## Тестування

## Усунення проблем
