# Zubax Telega ESCs

Zubax Telega is a high end, proprietary sensorless FOC motor control technology.
It is used in multiple products, including the [Zubax Myxa](https://zubax.com/products/myxa) ESC, [Zubax Mitochondrik](https://zubax.com/products/mitochondrik) motor controller module, and Zubax Sadulli integrated drive.

While Telega can be controlled using traditional PWM input, it is designed to operate over CAN bus using [DroneCAN](index.md).

:::info
ESCs based on Zubax Telega require non-trivial tuning of the propulsion system in order to deliver adequate performance and ensure robust operation.
Users who lack the necessary tuning expertise are advised to either [purchase pre-tuned UAV propulsion kits](https://zubax.com/products/uav_propulsion_kits) or to use Zubax Robotic's professional tuning service.
Questions on this matter should be addressed to: [support@zubax.com](mailto:support@zubax.com).
:::

![Sadulli - Top](../../assets/peripherals/esc_usavcan_zubax_sadulli/sadulli_top.jpg)

## 购买渠道

- [Zubax Myxa](https://zubax.com/products/myxa): High-end PMSM/BLDC motor controller (FOC ESC) for light unmanned aircraft and watercraft.
- [Zubax Mitochondrik](https://zubax.com/products/mitochondrik): Integrated sensorless PMSM/BLDC motor controller chip (used in ESCs and integrated drives)
- [Zubax Komar](https://shop.zubax.com/products/komar-motor-controller-open-hardware-reference-design-for-mitochondrik?variant=32931555868771): Open hardware reference design for Mitochondrik
- [Zubax Sadulli Integrated Drive](https://shop.zubax.com/collections/integrated-drives/products/sadulli-integrated-drive-open-hardware-reference-design-for-mitochondrik?variant=27740841181283)

## 硬件安装

ESCs are connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions. ESC order does not matter.

## Firmware Setup

Motor enumeration for [Telega-based ESCs](https://zubax.com/products/telega) is usually performed using the [Kucher tool](https://files.zubax.com/products/com.zubax.kucher/) (or less "GUI-friendly" [DroneCAN GUI Tool](https://dronecan.github.io/GUI_Tool/Overview/)).
Telega does NOT support automatic enumeration by spinning the motor.

There is some guidance here: [Quick start guide for Myxa v0.1](https://forum.zubax.com/t/quick-start-guide-for-myxa-v0-1/911) (Zubax blog).

Telega ESCs also require other motor setup and configuration for reliable performance. See the above guide and other Zubax documentation for more information.

## Flight Controller Setup

### Enable DroneCAN

Connect the ESCs to the Pixhawk CAN bus. Power up the entire vehicle using a battery or power supply (not just the flight controller over USB) and enable the DroneCAN driver by setting the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `3` to enable both dynamic node ID allocation and DroneCAN ESC output.

### PX4 配置

Assign motors to outputs using the [Acutator](../config/actuators.md#actuator-testing) configuration screen.

## 故障处理

### Motors not spinning when armed

If the PX4 Firmware arms but the motors do not start to rotate, check that parameter `UAVCAN_ENABLE=3` to use DroneCAN ESCs.
If the motors do not start spinning before thrust is increased, use [Acutator > Actuator Testing](../config/actuators.md#actuator-testing) to confirm that the motor outputs are set to the correct minimum values.

### DroneCAN devices dont get node ID/Firmware Update Fails

PX4 requires an SD card for DroneCAN node allocation and during firmware update (which happen during boot).
Check that there is a (working) SD card present and reboot.
