# PWM Servos and ESCs (Motor Controllers)

This section describes how to connect and power PWM-based brushless motor controllers and servos.

## ESC Connection Overview

Each PWM Electronic Speed Controller (ESC) minimally has the following wires:

- Power VBAT (usually thick and red)
- Power GND (usually thick and black)

And on the servo plug:

- PWM signal (usually white or yellow)
- GND (usually black or brown)

The servo plug _may_ also have a +5V wire (usually red or orange).
The purpose of this wire and how it is connected depends on particular ESC and vehicle type.

:::tip
In some cases (see below)the +5V line is not needed.
Instead of cutting the +5V line you can gently lift of the locking tab of the plastic housing of the servo connector for that pin (e.g. using a cutter blade or small screw driver) and pull the pin out.
Isolate it with electrical isolation tape and tape it to the servo cable.
This allows you to easily undo the wire later if needed
:::

## Power Connections

Always connect Power VBAT and GND to the battery, and the PWM signal and GND from the servo plug to the motor.

:::tip
There is **no setup** that does not require signal ground to be connected!
:::

The connection to the +5V wire (if present) depends on the ESC/Vehicle.

### Fixed-wing / VTOL

On a fixed-wing (or VTOL) ESC, the +5V line usually provides the output of a Battery Elimination Circuit (BEC).:

- This can be connected to the Pixhawk servo rail and used to power servos for flaps, ailerons etc.

  ::: info
  It is unsafe to power servos or ESCs from the autopilot's avionics power supply.
  This is why **Pixhawk series** flight controllers do not provide power for the servo rail (the AUX servo rail is unpowered and is limited to 1A).
  :::

- As a rule of thumb you should only connect the _output of only one BEC_ to the Pixhawk servo rail.
  (while it may be possible to connect multiple +5V outputs to the rail, this depends on the ESC model).

### Multicopter

On a multicopter, the +5V line might not be present or (if present) may not be connected.

- Multicopters often do not need servos, and hence do not need to power the Pixhawk servo rail (motors are usually separately powered from a power distribution board).
- There is no harm (or benefit) in connecting the wire to the servo rail.
- DJI ESCs typically include this wire, but it is not connected.

### Opto-isolated ESC

On an opto-isolated ESC **without** BEC, the +5V line might need to be connected and powered (in order to provide power to the ESC microcontroller).
In this case the wire will normally be connected to the flight controller servo rail, and the servo rail must be powered from an additional BEC.

## PX4 Configuration

PWM motors and servos are configured using the [Actuator Configuration](../config/actuators.md) screen in QGroundControl.

After assigning outputs and basic calibration, you may then wish to peform an [ESC Calibration](../advanced_config/esc_calibration.md).

Additional PX4 PWM configuration parameters can be found here: [PWM Outputs](../advanced_config/parameter_reference.md#pwm-outputs).

## Troubleshooting

Pixhawk is compatible with all _PWM ESCs_ on the market.
If a particular ESC is not operational, it is incorrectly wired up or configured.

### Ground Connection

Check that the ground (black wire) of the ESC servo connector is connected to Pixhawk (there is no valid wiring setup that does not have a ground reference).

:::warning
It is unsafe to fly without ground connected.
This is because for every positive pulse (the ESC signal) there needs to be an adjacent ground return path for a clean signal shape.

The image below shows how noisy the signal becomes if GND is not connected.

![PWM without ground](../../assets/hardware/pwm_esc/pwm_without_gnd.png)
:::

### Power Connection / Opto-isolated ESCs

If using an opto-isolated ESC that does not provide a BEC / power output, please ensure that the ESC does not need its +5V line powered for the opto-isolator.

See the first section of this page explains for other power connection considerations.

### Invalid Minimum Value

Some ESCs need to see a special low value pulse before switching on (to protect users who have the throttle stick in the middle position on power-up).

PX4 sends a pulse when the vehicle is disarmed, which silences the ESCs when they are disarmed and ensures that ESCs initialise correctly.
Appropriate values are determined and set as part of the [actuator configuration/testing](../config/actuators.md#actuator-testing) process (internally these set the per-output parameters [PWM_MAIN_DISn](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1) and [PWM_AUX_DISn](../advanced_config/parameter_reference.md#PWM_AUX_DIS1)).

### Timeout

Some ESCs may time out (preventing motor activation) if they have not received a valid low pulse within a few seconds of power on.

PX4 sends an idle/disarmed pulse right after power on to stop ESCs timing out.
Appropriate values are determined and set as part of the [actuator configuration/testing](../config/actuators.md#actuator-testing) process (internally these set the per-output parameters [PWM_MAIN_DISn](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1) and [PWM_AUX_DISn](../advanced_config/parameter_reference.md#PWM_AUX_DIS1)).

### Valid Pulse Shape, Voltage and Update Rate

::: info
This should not be a problem, but is included for completeness
:::

Pixhawk uses active high pulses, as used by all the major brands (Futaba, Spektrum, FrSky).

PWM interfaces are not formally standardised, however, the normal micro controllers all use TTL or CMOS voltage levels.
TTL is defined as low < 0.8V and high > 2.0V with some manufacturers using > 2.4V for additional noise margin.
CMOS logic is defined with similar voltage levels.
5V levels are **never** required to successfully switch to an _on_ state.

:::tip
Futaba, FrSky and Spektrum receivers output 3.3V or 3.0V voltage levels, as they are well above 2.4V.
Pixhawk has adopted this common industry pattern and outputs 3.3V levels on recent boards.
:::
