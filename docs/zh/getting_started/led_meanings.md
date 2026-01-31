# LED 含义（Pixhawk系列）

[Pixhawk-series flight controllers](../flight_controller/pixhawk_series.md) use LEDs to indicate the current status of the vehicle.

- The [UI LED](#ui_led) provides user-facing status information related to _readiness for flight_.
- The [Status LEDs](#status_led) provide status for the PX4IO and FMU SoC.
  它们表示电量、驱动模式和活动以及错误。

<a id="ui_led"></a>

## LED 界面

The RGB _UI LED_ indicates the current _readiness for flight_ status of the vehicle.
This is typically a superbright I2C peripheral, which may or may not be mounted on the flight controller board (i.e. FMUv4 does not have one on board, and typically uses an LED mounted on the GPS).

下图显示LED和飞行器状态的关系。

:::warning
It is possible to have a GPS lock (Green LED) and still not be able to arm the vehicle because PX4 has not yet [passed preflight checks](../flying/pre_flight_checks.md). **A valid global position estimate is required to takeoff!**
:::

:::tip
In the event of an error (blinking red), or if the vehicle can't achieve GPS lock (change from blue to green),   check for more detailed status information in _QGroundControl_ including calibration status, and errors messages reported by the [Preflight Checks (Internal)](../flying/pre_flight_checks.md).
还要检查GPS模块是否正确连接，Pixhawk是否正确读取GPS信息，GPS是否发送正确的GPS位置。
:::

![LED meanings](../../assets/flight_controller/pixhawk_led_meanings.gif)

- **[Solid Blue] Armed, No GPS Lock:** Indicates vehicle has been armed and has no position lock from a GPS unit.
  当飞行器已经解锁，PX4会解锁对电机的控制，允许你操纵无人机飞行。
  像往常一样，在解锁时要小心，因为大型螺旋桨在高速旋转时可能很危险。
  飞行器在这种模式下无法执行引导任务。

- **[Pulsing Blue] Disarmed, No GPS Lock:** Similar to above, but your vehicle is disarmed.
  这意味着你将不能控制电机，但是其他子系统正在工作。

- **[Solid Green] Armed, GPS Lock:** Indicates vehicle has been armed and has a valid position lock from a GPS unit.
  当飞行器已经解锁，PX4会解锁对电机的控制，允许你操纵无人机飞行。
  像往常一样，在解锁时要小心，因为大型螺旋桨在高速旋转时可能很危险。
  在这种模式下，飞行器可以执行引导任务。

- **[Pulsing Green] Disarmed, GPS Lock:** Similar to above, but your vehicle is disarmed.
  这意味着你讲无法控制电机，但是其他子系统包括GPS位置锁正在工作。

- **[Solid Purple] Failsafe Mode:** This mode will activate whenever vehicle encounters an issue during flight,
  such as losing manual control, a critically low battery, or an internal error.
  在故障保护模式时，飞行器将试图返回起飞位置，或者降落在当前位置。

- **[Solid Amber] Low Battery Warning:** Indicates your vehicle's battery is running dangerously low.
  在某一点之后，飞行器将进入故障保护模式。 但是，此模式警告此次飞行应该结束。

- **[Blinking Red] Error / Setup Required:** Indicates that your autopilot needs to be configured or calibrated before flying.
  将飞行器连接到地面站以找出问题所在。
  如果您已经完成设置过程，飞行器仍然闪烁红色，这表明还有其他错误。

<a id="status_led"></a>

## LED 状态

Three _Status LEDs_ provide status for the FMU SoC, and three more provide status for the PX4IO (if present).
它们表示电量、驱动模式和活动以及错误。

![Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_status_leds.jpg)

从上电开始，FMU和PX4IO的CPU首先运行引导程序(BL) 然后运行程序(APP)。
下表显示了Bootloader 和 APP 如何使用 LED 指示状态。

| 颜色     | 标签                                 | 引导加载程序使用        | APP使用   |
| ------ | ---------------------------------- | --------------- | ------- |
| 蓝色     | ACT(激活)         | 引导加载程序接收数据的时候闪烁 | 表示ARM状态 |
| 红色/琥珀色 | B/E(在引导加载程序/错误) | 在引导加载程序时闪烁      | 表示错误状态  |
| 绿色     | PWR(电源)         | 引导加载程序不使用       | 表示ARM状态 |

:::info
The LED labels shown above are commonly used, but might differ on some boards.
:::

下面给出了LED更详细的信息(“x”表示任意状态)

| 红色/琥珀色 | 蓝色 | 绿色    | 含义                                                                                                                                                                                   |
| ------ | -- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 10Hz   | x  | x     | Overload CPU load > 80%, or RAM usage > 98%                                                                                                                                          |
| 关闭     | x  | x     | Overload CPU load <= 80%, or RAM usage <= 98%                                                                                      |
| 不可用    | 关闭 | 4 赫兹  | actuator_armed->armed && failsafe                                                                                       |
| 不可用    | 打开 | 4 赫兹  | actuator_armed->armed && !failsafe                                                                                      |
| 不可用    | 关闭 | 1 赫兹  | !actuator_armed-> armed && actuator_armed->ready_to_arm  |
| 不可用    | 关闭 | 10 赫兹 | !actuator_armed->armed  && !actuator_armed->ready_to_arm |
