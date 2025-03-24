# 조이스틱 설정

A [computer joystick](https://en.wikipedia.org/wiki/Joystick) or gamepad connected through _QGroundControl_ can be used to manually control the vehicle (_instead_ of using an [RC Transmitter](../config/radio.md)).

This approach may be used by manual control units that have an integrated ground control station (like the _UAVComponents_ [MicroNav](https://uxvtechnologies.com/ground-control-stations/micronav/) shown below).
조이스틱은 일반적으로 비행 시뮬레이션용으로 많이 사용합니다.

![Joystick MicroNav](../../assets/peripherals/joystick/micronav.jpg)

:::tip
[Radio Setup](../config/radio.md) is not required if using only a joystick (because a joystick is not an RC controller)!
:::

:::info
_QGroundControl_ uses the cross-platform [SDL2](http://www.libsdl.org/index.php) library to convert joystick movements to MAVLink [MANUAL_CONTROL](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) messages, which are then sent to PX4 over the telemetry channel.
결과적으로 조이스틱 기반 제어 시스템은 차량이 조이스틱 움직임에 반응하기 위해 안정적인 고대역폭 원격 채널이 필요합니다.
:::

## PX4 조이스틱 지원 활성화

Information about how to set up a joystick is covered in: [QGroundControl > Joystick Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html).

요약

- Open _QGroundControl_
- Set the parameter [COM_RC_IN_MODE=1](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) - `Joystick`
  - See [Parameters](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/parameters.html) for information about setting parameters
  - Setting the parameter to `2` or `3` also enables Joystick under some circumstances.
- 조이스틱을 연결합니다.
- Configure the connected joystick in: **Vehicle Setup > Joystick**.
