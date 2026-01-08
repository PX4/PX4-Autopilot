# Vertiq All-In-One Motor/ESC Modules

Vertiq makes high performance propulsion systems for commercial and defense UAS.
The core design consists of a lightweight, tightly integrated motor and ESC with an embedded position sensor.
With closed loop velocity control for the fastest response times available, class leading efficiency, no startup and reverse jitter allowing low speed control and smooth reversibility, and a built in "stow" controller for smoothly placing idle propellers in a preferred direction, Vertiq's modules have significant advantages over other ESCs.

![Vertiq Module Lineup](../../assets/peripherals/esc_vertiq/vertiq_esc_lineup.jpg)

All Vertiq modules support traditional [PWM input, DShot, OneShot, and Multishot communication protocols](https://iqmotion.readthedocs.io/en/latest/communication_protocols/hobby_protocol.html). Vertiq's larger modules also support [DroneCAN control](https://iqmotion.readthedocs.io/en/latest/communication_protocols/dronecan_protocol.html).

## Where to Buy

Purchasing information can be found on the [Vertiq website](https://www.vertiq.co/).

## Hardware Setup

### Wiring

Connecting your Vertiq module to a PWM output from your flight controller or DroneCAN bus will vary depending on your model.
Please see the product data sheets for wiring information.

All Vertiq datasheets can be found at [vertiq.co](https://www.vertiq.co/).

## Firmware Setup

The best tool to configure your Vertiq module is Vertiq's IQ Control Center application.
You can find instructions for installation in [Getting Started with Speed Modules Using IQ Control Center](https://iqmotion.readthedocs.io/en/latest/control_center_docs/speed_module_getting_started.html).

To get started with traditional PWM input or DShot with your flight controller, please see [PWM and DSHOT Control with a Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/pwm_control_flight_controller.html).

To get started with DroneCAN with your flight controller, please see [DroneCAN Integration with a PX4 Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/dronecan_flight_controller.html).

## Flight Controller Setup

### DroneCAN Configuration

Instructions for integrating the motor/ESC using with DroneCAN can be found in [Flight Controller Configuration](https://iqmotion.readthedocs.io/en/latest/tutorials/dronecan_flight_controller.html#dronecan-integration-with-a-px4-flight-controller) (in _DroneCAN Integration with a PX4 Flight Controller_).

These instructions walk you through setting the correct parameters for enabling the flight controller's DroneCAN drivers, setting the correct configuration parameters for communication with Vertiq modules on the DroneCAN bus, ESC configuration, and testing that your flight controller can properly control your modules over DroneCAN.

#### LED Configuration for Vertiq Modules

::: info
This configuration is only required if you have the optional [Vertiq LED module add-on](https://www.vertiq.co/add-ons).
Standard Vertiq ESC modules do not include LEDs.
:::

Vertiq modules with the LED add-on have two addressable LEDs per ESC: an RGB LED for status colours and a White LED for anti-collision lighting.
The UAVCAN driver automatically calculates the correct `light_id` values using the formula: `light_id = esc_index * 3 + BASE_ID` (where RGB uses BASE_ID=1, White uses BASE_ID=2).

To enable LED control for Vertiq modules:

1. Set [UAVCAN_LGT_MODE](../advanced_config/parameter_reference.md#UAVCAN_LGT_MODE) to `1` (Vertiq).
2. Reboot the flight controller.

The driver will automatically detect the number of connected ESCs from the `esc_status` topic and send LED commands to all ESCs.
RGB LEDs display the standard PX4 status colours (arming state, errors, etc.), while White LEDs are controlled by the anti-collision light settings.


### DShot/PWM Configuration

Instructions for integrating the motor/ESC using PWM and DShot can be found in [PWM and DShot Control with a Flight Controller](https://iqmotion.readthedocs.io/en/latest/tutorials/pwm_control_flight_controller.html).
DShot is recommended.

## Further Information

- <https://www.vertiq.co/> — Learn more about Vertiq modules
- [Vertiq Documentation](https://iqmotion.readthedocs.io/en/latest/index.html) — Additional information about configuring your Vertiq module
