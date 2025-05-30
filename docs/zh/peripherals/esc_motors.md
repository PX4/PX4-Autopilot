# ESCs & Motors

Many PX4 drones use brushless motors that are driven by the flight controller via an Electronic Speed Controller (ESC).
The ESC takes a signal from the flight controller and uses it to set control the level of power delivered to the motor.

PX4 supports a number of common protocols for sending the signals to ESCs: [PWM ESCs](../peripherals/pwm_escs_and_servo.md), [OneShot ESCs](../peripherals/oneshot.md), [DShot ESCs](../peripherals/dshot.md), [DroneCAN ESCs](../dronecan/escs.md), PCA9685 ESC (via I2C), and some UART ESCs (from Yuneec).

有关详细信息，请参阅︰

- [PWM ESCs and Servos](../peripherals/pwm_escs_and_servo.md)
- [OneShot ESCs and Servos](../peripherals/oneshot.md)
- [DShot](../peripherals/dshot.md)
- [DroneCAN ESCs](../dronecan/escs.md)
- [ESC Calibration](../advanced_config/esc_calibration.md)
- [ESC Firmware and Protocols Overview](https://oscarliang.com/esc-firmware-protocols/) (oscarliang.com)

A high level overview of the main ESC/Servo protocols supported by PX4 is given below.

## ESC Protocols

### PWM

[PWM ESCs](../peripherals/pwm_escs_and_servo.md) are commonly used for fixed-wing vehicles and ground vehicles (vehicles that require a lower latency like multicopters typically use oneshot or dshot ESCs).

PWM ESCs communicate using a periodic pulse, where the _width_ of the pulse indicates the desired power level.
The pulse wdith typically ranges between 1000uS for zero power and 2000uS for full power.
The periodic frame rate of the signal depends on the capability of the ESC, and commonly ranges between 50Hz and 490 Hz (the theoretical maximum being 500Hz for a very small "off" cycle).
A higher rate is better for ESCs, in particular where a rapid response to setpoint changes is needed.
For PWM servos 50Hz is usually sufficient, and many don't support higher rates.

![duty cycle for PWM](../../assets/peripherals/esc_pwm_duty_cycle.png)

In addition to being a relatively slow protocol PWM ESCs require [calibration](../advanced_config/esc_calibration.md) because the range values representing low and high values can vary significantly.
Unlike [dshot](#dshot) and [DroneCAN ESC](#dronecan) they do not have the ability to provide telemetry and feedback on ESC (or servo) state.

Setup:

- [ESC Wiring](../peripherals/pwm_escs_and_servo.md)
- [PX4 Configuration](../peripherals/pwm_escs_and_servo.md#px4-configuration)
- [ESC Calibration](../advanced_config/esc_calibration.md)

### Oneshot 125

[OneShot 125 ESCs](../peripherals/oneshot.md) are usually much faster than PWM ESCs, and hence more responsive and easier to tune.
They are preferred over PWM for multicopters (but not as much as [DShot ESCs](#dshot), which do not require calibration, and may provide telemetry feedback).
There are a number of variants of the OneShot protocol, which support different rates.
PX4 only supports OneShot 125.

OneShot 125 is the same as PWM but uses pulse widths that are 8 times shorter (from 125us to 250us for zero to full power).
This allows OneShot 125 ESCs to have a much shorter duty cycle/higher rate.
For PWM the theoretical maximum is close to 500 Hz while for OneShot it approaches 4 kHz.
The actual supported rate depends on the ESC used.

Setup:

- [ESC Wiring](../peripherals/pwm_escs_and_servo.md) (same as for PWM ESCs)
- [PX4 Configuration](../peripherals/oneshot.md#px4-configuration)
- [ESC Calibration](../advanced_config/esc_calibration.md)

### DShot

[DShot](../peripherals/dshot.md) is a digital ESC protocol that is highly recommended for vehicles that can benefit from reduce latency, in particular racing multicopters, VTOL vehicles, and so on.

It has reduced latency and is more robust than both [PWM](#pwm) and [OneShot](#oneshot-125).
In addition it does not require ESC calibration, telemetry is available from some ESCs, and you can revers motor spin directions

PX4 configuration is done in the [Actuator Configuration](../config/actuators.md).
Selecting a higher rate DShot ESC in the UI result in lower latency, but lower rates are more robust (and hence more suitable for large aircraft with longer leads); some ESCs only support lower rates (see datasheets for information).

Setup:

- [ESC Wiring](../peripherals/pwm_escs_and_servo.md) (same as for PWM ESCs)
- [DShot](../peripherals/dshot.md) also contains information about how to send commands etc.

### DroneCAN

[DroneCAN ESCs](../dronecan/escs.md) are recommended when DroneCAN is the primary bus used for your vehicle.
The PX4 implementation is currently limited to update rates of 200Hz.

DroneCAN shares many similar benefits to [Dshot](#dshot) including high data rates, robust connection over long leads, telemetry feedback, no need for calibration of the ESC itself.

[DroneCAN ESCs](../dronecan/escs.md) are connected via the DroneCAN bus (setup and configuration are covered at that link).
