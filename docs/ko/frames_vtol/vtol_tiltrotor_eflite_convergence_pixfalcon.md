# E-flite Convergence 틸트로터 VTOL (Pixfalcon)

The [E-Flite Convergence](https://youtu.be/HNedXQ_jhYo) can easily be converted to a fully autonomous VTOL with PX4.
There is not much space but it's enough for a [Pixfalcon](../flight_controller/pixfalcon.md) flight controller with GPS and telemetry.

:::info
The original Horizon Hobby _E-Flite Convergence_ frame and [Pixfalcon](../flight_controller/pixfalcon.md) have been discontinued.
Alternatives are provided in the [Purchase](#where-to-buy) section.
:::

<lite-youtube videoid="E61P2f2WPNU" title="E-flite Convergence Autonomous Mission Flight"/>

## 구매처

차량 프레임 옵션:

- **WL Tech XK X450** - [AliExpress](https://www.aliexpress.com/item/1005001946025611.html)
- **JJRC M02** - [Banggood (AU)](https://au.banggood.com/JJRC-M02-2_4G-6CH-450mm-Wingspan-EPO-Brushless-6-axis-Gyro-Aerobatic-RC-Airplane-RTF-3D-or-6G-Mode-Aircraft-p-1588201.html), [AliExpress](https://www.aliexpress.com/item/4001031497018.html)

비행 컨트롤러 옵션():

- [Pixhawk 4 Mini](../flight_controller/pixhawk4_mini.md)
- [Holybro Pixhawk Mini](../flight_controller/pixhawk_mini.md).
- 충분히 작은 폼 팩터를 가진 다른 호환 가능한 비행 콘트롤러.

## 하드웨어 설정

The vehicle needs 7 PWM signals for the motors and control surfaces:

- Motor (left/right/back)
- Tilt servos (right/left)
- Elevons (left/right)

These can be wired to flight controller outputs more-or-less in any way you want (though outputs for motors should be grouped together, and so on).

The outputs are configured in the [Actuators Configuration](../config/actuators.md) by following the instructions for VTOL tiltrotor geometry and output configuration.
Note that you will need to start from the [Generic Tiltrotor VTOL](../airframes/airframe_reference.md#vtol_vtol_tiltrotor_generic_tiltrotor_vtol) frame.

Note that left and right in the configuration screen and frame reference are defined from the perspective of a human pilot inside a real plane (or looking from above, as shown below):

<img src="../../assets/airframes/types/VTOLTiltRotor_eflite_convergence.svg" width="300px" />

### 비행 콘트롤러

비행 콘트롤러는 원래 자동조종장치가 있던 동일한 위치에 장착할 수 있습니다.

![Mount Pixfalcon](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_pixfalcon_mounting.jpg)

### 텔레메트리

텔레메트리 모듈은 FPV 전송 기어를 고정하기 위한 베이에 장착합니다.

![Mount telemetry module](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_telemetry_module.jpg)

### GPS

GPS 장착을 위하여 "조종실" 내부의 발포 재질 부분을 잘라냈습니다.
그렇게 하면 GPS를 차체 내부에 넣을 수 있고, 차량 외관을 손상시키지 않으면서 멋지게 수납할 수 있습니다.

![Mount GPS](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_gps_mounting.jpg)

## PX4 설정

Follow the [Standard Configuration](../config/index.md) in _QGroundControl_ (radio, sensors, flight modes, etc.).

이 기체와 관련된 특정 설정은 다음과 같습니다.

- [Airframe](../config/airframe.md)
  - Select the airframe configuration **E-flite Convergence** under **VTOL Tiltrotor** and restart _QGroundControl_.
    ![QGroundControl Vehicle Setting - Airframe selection E-Flight](../../assets/airframes/vtol/eflite_convergence_pixfalcon/qgc_setup_airframe.jpg)
- [Flight Modes/Switches](../config/flight_mode.md)
  - As this is a VTOL vehicle, you must [assign an RC controller switch](../config/flight_mode.md#what-flight-modes-and-switches-should-i-set) for transitioning between multicopter and fixed-wing modes.
