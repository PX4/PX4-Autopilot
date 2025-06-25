# DShot 电调

DShot is an alternative ESC protocol that has several advantages over [PWM](../peripherals/pwm_escs_and_servo.md) or [OneShot](../peripherals/oneshot.md):

- 低延迟
- 通过校验提高鲁棒性
- 由于协议使用数字编码，因此无需电调校准
- 很多电调支持数传反馈。
- 可以通过命令反转电机方向（而不是物理调换两根线/重新焊接）。
- 支持其他好用的命令。

本章介绍了如何连接和配置 DShot 电调。

## Wiring/Connections {#wiring}

DShot ESC are wired the same way as [PWM ESCs](pwm_escs_and_servo.md).
唯一的区别在于它们只能连接到FMU，通常只能连接到某些引脚的子集。

:::info
You may want to check the actuator configuration screen to see what pins are available for DShot on your controller before wiring up!
:::

Pixhawk controllers with both an FMU and an IO board usually label them as `AUX` (FMU) and `MAIN` (IO), respectively.
These match the `PWM AUX` and `PWM MAIN` output tabs on the actuator configuration screen.
For these controllers connect the DShot ESC to the `AUX` port.

Controllers that don't have an IO board usually label the (single) output port as `MAIN`, and this is where you will connect your DShot ESC.
If the controller without IO has its own firmware, the actuator assignment will be to the matching `PWM MAIN` outputs.
However if the same firmware is used for hardware with/without the IO board, such as for the Pixhawk 4 and Pixhawk 4 Mini, then actuator assignment tab used is the same in both cases: `PWM AUX` (i.e. not matching the port label `MAIN` in the "mini" case).

## 配置

:::warning
Remove propellers before changing ESC configuration parameters!
:::

Enable DShot for your required outputs in the [Actuator Configuration](../config/actuators.md).

DShot comes with different speed options: _DShot150_, _DShot300_, and _DShot600_ where the number indicates the speed in kilo-bits/second.
您应该将参数设置为您的电调支持的最高速度（根据其说明书）。

然后连接电池并解锁无人机。
电调应该初始化，电机应该按照正确的方向转动。

- If the motors do not spin in the correct direction (for the [selected airframe](../airframes/airframe_reference.md)) you can reverse them in the UI using the **Set Spin Direction** option (this option appears after you select DShot and assign motors).
  You can also reverse motors by sending an [ESC Command](#commands).

## ESC Commands {#commands}

Commands can be sent to the ESC via the [MAVLink shell](../debug/mavlink_shell.md).
See [here](../modules/modules_driver.md#dshot) for a full reference of the supported commands.

其中最重要的是：

- Make a motor connected to to FMU output pin 1 beep (helps with identifying motors)

  ```sh
  dshot beep1 -m 1
  ```

- Retrieve ESC information (requires telemetry, see below):

  ```sh
  nsh> dshot esc_info -m 2
  INFO  [dshot] ESC Type: #TEKKO32_4in1#
  INFO  [dshot] MCU Serial Number: xxxxxx-xxxxxx-xxxxxx-xxxxxx
  INFO  [dshot] Firmware version: 32.60
  INFO  [dshot] Rotation Direction: normal
  INFO  [dshot] 3D Mode: off
  INFO  [dshot] Low voltage Limit: off
  INFO  [dshot] Current Limit: off
  INFO  [dshot] LED 0: unsupported
  INFO  [dshot] LED 1: unsupported
  INFO  [dshot] LED 2: unsupported
  INFO  [dshot] LED 3: unsupported
  ```

- Permanently set the spin direction of a motor connected to FMU output pin 1 (while motors are _not_ spinning):

  - Set spin direction to `reversed`:

    ```sh
    dshot reverse -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: reversed
    ```

  - Set spin direction to `normal`:

    ```sh
    dshot normal -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: normal
    ```

  :::note

  - The commands will have no effect if the motors are spinning, or if the ESC is already set to the corresponding direction.
  - The ESC will revert to its last saved direction (normal or reversed) on reboot if `save` is not called after changing the direction.


:::

## ESC Telemetry

Some ESCs are capable of sending telemetry back to the flight controller through a UART RX port.
这些DShot 电调会有一条额外的连接线。

The provided telemetry includes:

- Temperature
- Voltage
- Current
- Accumulated current consumption
- 转速值

开启遥测数据功能（仅对于能支持遥测数据的电调）:

1. 把电调上的TELE端口连接到飞控上空的串口的 RX 端。
2. Enable telemetry on that serial port using [DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG).

重启后，您可以检查TELE数据传输是否正常工作（确保电池已连接），方法如下：

```sh
dshot esc_info -m 1
```

:::tip
You may have to configure [MOT_POLE_COUNT](../advanced_config/parameter_reference.md#MOT_POLE_COUNT) to get the correct RPM values.
:::

:::tip
Not all DSHOT-capable ESCs support `[esc_info]`(e.g. APD 80F3x), even when telemetry is supported and enabled.
显示的错误是：

```sh
ERROR [dshot] No data received. If telemetry is setup correctly, try again.
```

查看制造商文档以获取详细信息。
:::

## Bidirectional DShot (Telemetry)

<Badge type="tip" text="PX4 v1.16" />

Bidirectional DShot is a protocol that can provide telemetry including: high rate ESC RPM data, voltage, current, and temperature with a single wire.

The PX4 implementation currently enables only ESC RPM (eRPM) data collection from each ESC at high frequencies.
This telemetry significantly improves the performance of [Dynamic Notch Filters](../config_mc/filter_tuning.md#dynamic-notch-filters) and enables more precise vehicle tuning.

:::info
The [ESC Telemetry](#esc-telemetry) described above is currently still necessary if you want voltage, current, or temperature information.
It's setup and use is independent of bidirectional DShot.
:::

### 硬件安装

The ESC must be connected to FMU outputs only.
These will be labeled `MAIN` on flight controllers that only have one PWM bus, and `AUX` on controllers that have both `MAIN` and `AUX` ports (i.e. FCs that have an IO board).

:::warning
**Limited hardware support**
This feature is only supported on flight controllers with the following processors:

- STM32H7: First four FMU outputs
  - Must be connected to the first 4 FMU outputs, and these outputs must also be mapped to the same timer.
  - [KakuteH7](../flight_controller/kakuteh7v2.md) is not supported because the outputs are not mapped to the same timer.
- [i.MXRT](../flight_controller/nxp_mr_vmu_rt1176.md) (V6X-RT & Tropic): 8 FMU outputs.

No other boards are supported.
:::

### Configuration {#bidirectional-dshot-configuration}

To enable bidirectional DShot, set the [DSHOT_BIDIR_EN](../advanced_config/parameter_reference.md#DSHOT_BIDIR_EN) parameter.

The system calculates actual motor RPM from the received eRPM data using the [MOT_POLE_COUNT](../advanced_config/parameter_reference.md#MOT_POLE_COUNT) parameter.
This parameter must be set correctly for accurate RPM reporting.
