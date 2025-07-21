# OneShot Servos and ESCs (Motor Controllers)

PX4 support OneShot 125 ESCs (only).
These are typically faster and more responsive than [PWM ESCs](../peripherals/pwm_escs_and_servo.md) but share the same wiring setup (all you need to do is set some different parameters)

:::info
[DShot](../peripherals/dshot.md) should always be used instead of OneShot where possible, as it is more responsive, more robust, does not required calibration, and may support telemetry.
The only reason not to use DShot would be hardware limitations (insufficient DShot pins available or using an ESC that does not support DShot).
:::

## 개요

OneShot is essentially a version of [PWM](../peripherals/pwm_escs_and_servo.md) that can be, in theory, up to 8 times faster.

Both PWM and OneShot communicate using a periodic pulse, where the width of the pulse indicates the desired power level.
For PWM the pulse length typically ranges between 1000us (zero) and 2000us (full power), while for OneShot 125 the pulse widths are 8 times shorter, ranging from 125us (zero power) to 250us (full power).

The theoretical maximum rate at which pulses can be sent, and hence the responsiveness, depends on the width of the largest pulse.
For PWM this rate is close to 500 Hz while for OneShot it approaches 4 kHz.
In practice the actual maximum rate for OneShot ESCs is typically between 1 kHz and 2 kHz, depending on the ESC used.

## 설정

### 배선

Wiring is exactly the same as for [PWM ESCs](../peripherals/pwm_escs_and_servo.md) (and dshot).

### PX4 설정

To enable OneShot select the protocol for a group of outputs during [Actuator Configuration](../config/actuators.md).
Note that the output range values are set to values in the normal PWM range (nominally `1000` to `2000`).
These are scaled internally to output appropriate pulse-widths for Oneshot.

Then perform [ESC Calibration](../advanced_config/esc_calibration.md).
