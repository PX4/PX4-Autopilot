# Discontinued: Falcon Vertigo Hybrid VTOL RTF (Dropix)

:::warning
Discontinued
The Falcon Venturi FPV Wing frame on which this vehicle is based is no longer available.
:::

The _Falcon Vertigo Hybrid VTOL_ is a quadplane VTOL aircraft that has been designed to work with PX4 and the Dropix (Pixhawk compatible) flight controller. It can carry a small GoPro camera.

The RTF kit contains everything needed for a full system except an RC receiver and telemetry module.
The components can also be purchased separately.

Key information:

- **Frame:** Falcon Vertigo Hybrid VTOL
- **Flight controller:** Dropix
- **Wing span:** 1.3m

![Falcon Vertigo Hybrid VTOL RTF](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_complete.jpg)

## Bill of Materials

Almost everything you need is provided in the RTF kit (the links next to components below are provided in case you wished to purchase any component separately):

- Pre laminated EPP wings
- Wingtips and full hardware
- Dropix flight controller (discontinued) with
  - GPS u-blox M8N
  - Power Sensor
  - [Airspeed Sensor](https://store-drotek.com/793-digital-differential-airspeed-sensor-kit-.html)
- Quad power set  [Tiger Motor MT-2216-11 900kv V2](https://www.getfpv.com/tiger-motor-mt-2216-11-900kv-v2.html) (discontinued)
- 4 x propeller 10”x 5” (quad motors)
- 4 x [ESC 25A](http://www.getfpv.com/tiger-motor-flame-25a-esc.html)
- 1 x propeller 10” x 5” (pusher motor)
- 1 x ESC 30A
- Pusher motor power system
- Carbon fiber tubes and mounts
- G10 motor mounts
- 1 x [3700mah 4S 30C Lipo battery](https://www.overlander.co.uk/batteries/lipo-batteries/power-packs/3700mah-4s-14-8v-25c-lipo-battery-overlander-sport.html)
- Dropix power distribution board and cable

The kit does not come with a radio receiver or (optional) telemetry modules.
For this build we used the following components:

- Receiver: [FrSSKY D4R-II](https://www.frsky-rc.com/product/d4r-ii/)
- Telemetry: [Holybro 100mW 915MHz modules](https://www.getfpv.com/holybro-100mw-fpv-transceiver-telemetry-radio-set-915mhz.html) (Discontinued)

## Tools needed

The following tools were used to assemble the airframe:

- Philips screwdriver
- 5.5 mm Hex Socket Screwdriver
- Wire cutters
- Soldering iron and solder
- Hobby stainless steel tweezer
- Gorilla glue
- Fiberglass reinforced tape

![Build tools](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_build_tools.jpg)

## Assembly steps

The RTF kit requires the following assembly.

### Step 1: Attach motors mounts

1. Spread gorilla glue inside the wing brackets as shown.

  ![Add glue on wing brackets](../../assets/airframes/vtol/falcon_vertigo/wing_brackets_glue.jpg)

2. Attach the carbon tube in the brackets. The bracket and tube must be aligned using the white mark (as shown in the picture).

  ::: info
  This is very important because the white mark indicates the center of gravity.

:::

  <img src="../../assets/airframes/vtol/falcon_vertigo/carbon_tube_in_brackets.jpg" title="Carbon tube in brackets" width="300px" />

3. The following images show the alignment of rods from other viewpoints:

  ![quad motor frame rod alignment from bottom](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_9_bottom_view_rod_alignment.jpg)
  ![quad motor frame rod alignment schematic](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_11_rod_alignment_schamatic.jpg)

### Step 2: Attach the wings

1. Insert both carbon tubes into the fuselage.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_15_fuselage_tubes.jpg" width="500px" title="Fuselage carbon tubes" />

2. Spread gorilla glue between the two white marks on each tube (indicated by the red arrows). The white mark in the center (blue arrow) will be placed in the center of the fuselage and the other marks on the sides.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_13_rod_apply_glue.jpg" width="500px" title="Apply glue to rod" />

3. Once the carbon tubes are inside the fuselage, spread gorilla glue on the rest of the tube and attach the wings.

4. The fuselage has two holes for the motor and servo cables. Pass the cables through the holes and then join the wings to the fuselage.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_17_fuselage_holes_cables.jpg" width="500px" title="Fuselage holes for cables" />

5. Within the fuselage connect the signal cables you just passed through from the wings to the ESC using the provided connectors. The ESC are already connected to the motors and set up to turn in the correct order (you will need to connect the ESC PDB to a power module in a later step).

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_19_connect_esc_power_and_signal_cables.jpg" width="500px" title="Connect ESC power and signal cables" />

6. As with the ESCs, the servos are already installed. Connect the signal cable from the wing (passed through the fuselage) to the flight controller.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_21_connect_servo_cables.jpg" width="500px" title="Connect servo cables" />

7. Repeat these steps for the other wing.

### Step 3: Connect the electronics

This kit includes Dropix flight controller with most of the required electronics pre-connected (if you use another Pixhawk-compatible flight controller the connections are similar).

<img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_23_dropix_and_other_electronics.jpg" width="500px" title="Falcon Vertigo Electronics" />

:::info
General information about connecting Dropix can be found in [Dropix Flight Controller](../flight_controller/dropix.md).
:::

#### Connect the ESC power connector and pass the signals cables to the flight controller

1. Connect the ESC to the power module using the XT60 connector

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_25_aileron_esc_connections.jpg" width="500px" title="" />

2. Pass the signals cables through to the flight controller

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_27_gps_esc_servo_connections.jpg" width="500px" title="GPS, ESC, Servo connections" />

#### Motor Wiring

Motor and servo wiring is nearly entirely up to you, but should match the [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol) configuration, as shown in the airframe reference.
The geometry and output assignment can be configured in the [Actuators Configuration](../config/actuators.md#actuator-outputs)

For example, you might wire it up like this example (orientation as if "sitting in the plane"):

| Port   | 接口定义                   |
| ------ | ---------------------- |
| MAIN 1 | Front right motor, CCW |
| MAIN 2 | Back left motor, CCW   |
| MAIN 3 | Front left motor, CW   |
| MAIN 4 | Back right motor, CW   |
| AUX  1 | Left aileron           |
| AUX  2 | Right aileron          |
| AUX  3 | Elevator               |
| AUX  4 | Rudder                 |
| AUX  5 | 油门                     |

<a id="dropix_back"></a>

#### Flight Controller Connections: Motors, Servos, RC receiver, current sensor

The image below shows back of the dropix flight controller, highlighting the outputs pins to connect quad motors cables, aileron signal cables, throttle motor, and the current sensor and receiver (RC IN) input pins.

<img id="dropix_outputs" src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_33_dropix_outputs.jpg" width="500px" title="Dropix motor/servo outputs" />

1. Connect quad motors signal cables.

2. Connect the aileron cables and throttle motor in the auxiliary outputs.

3. Connect the throttle motor signal cable from the ESC to the appropriate flight controller auxiliary port. Connect the ESC to the throttle motor.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_37_connect_throttle_motor.jpg" width="500px" title="Connect throttle motor" />

4. Connect the receiver (RC IN).

<a id="dropix_front"></a>

#### Flight Controller Connections: Telemetry, Airspeed Sensor, GPS, Buzzer and Safety Switch

The sensor inputs, telemetry, buzzer and safety switch are located in the front of the flight controller, as shown in the connection diagram below.

<img src="../../assets/flight_controller/dropix/dropix_connectors_front.jpg" width="500px" alt="Dropix connectors front" title="Dropix connectors front" />

1. Connect the telemetry, airspeed sensor, GPS, buzzer and safety switch as shown.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_39_connect_sensors.jpg" width="500px" title="Connect sensors" />

#### Flight Controller: Connect power module and external USB

The inputs for the USB port, power module and external USB are located on the right side of the flight controller.

1. Connect power and USB as shown

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_41_connect_power_module_usb.jpg" width="500px" title="Connect power module and USB" />

:::tip
The external USB is optional.
It should be used if access to the USB port is difficult once the flight controller is mounted.
:::

#### Install the pitot tube (airspeed sensor)

The pitot tube is installed on the front of the plane and connected to the airspeed sensor via a tube.

:::warning
It is important that nothing obstructs airflow to the Pitot tube. This is critical for fixed-wing flight and for transitioning from quad to plane.
:::

1. Install the Pitot tube in the front of the plane

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_43_airspeed_sensor_mounting.jpg" width="500px" title="Airspeed sensor mounting" />

2. Secure the connecting tubing and ensure that it is not bent/kinked.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_45_airspeed_sensor_tubing.jpg" width="500px" title="Airspeed sensor mounting" />

3. Connect the tubes to the airspeed sensor.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_47_connect_airspeed_sensor_tubing.jpg" width="500px" title="Connect airspeed sensor and tubing" />

#### Install/connect receiver and telemetry module

1. Paste the receiver and telemetry module to the outside of the vehicle frame.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_49_receiver_mounting.jpg" width="500px" title="Paste receiver" />

2. Connect the receiver to the RC IN port on the _back_ of the dropix, as shown above (also see the [flight controller instructions](#dropix_back)).

3. Connect the telemetry module to the _front_ of the flight controller as shown below (see the [flight controller instructions](#dropix_front) for more detail on the pins).

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_51_telemetry_module_mounting.jpg" width="500px" title="Paste telemetry module" />

<a id="compass_gps"></a>

#### GPS/Compass module

The GPS/Compass module is already mounted on the wing, in the default orientation. You don't need to have to do anything extra for this!

<img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_gps_compass.jpg" width="500px" title="GPS/Compass" />

<a id="flight_controller_orientation"></a>

#### Mount and orient the flight controller

1. Set your flight controller orientation to 270 degrees.

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_53_flight_controller_orientation.jpg" width="500px" title="Flight controller orientation" />

2. Secure the controller in place using vibration damping foam.

### Step 4: Final Assembly Checks

The final assembly step is to check the vehicle is stable and that the motors have been set up correctly.

1. Check that the motors turn in the correct directions (as in the QuadX diagram below).

  <img src="../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_35_quad_motor_directions.png" width="200px" title="Quad motor order/directions" />

  ::: info
  If necessary the servo direction can be reversed using the `Rev Range (for servos)` checkbox associated with each servo output in the QGroundControl [Actuator Output](../config/actuators.md#actuator-outputs) configuration (for servos only) (this sets the [PWM_AUX_REV](../advanced_config/parameter_reference.md#PWM_AUX_REV) or [PWM_AUX_MAIN](../advanced_config/parameter_reference.md#PWM_MAIN_REV) parameter).

:::

2. Check the vehicle is balanced around the expected centre of gravity

  - Hold the vehicle with your fingers at the center of gravity and check that the vehicle remains stable.

    ![Level Centre of Gravity](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_57_level_centre_of_gravity.jpg)

  - If the vehicle leans forward or backwards, move the motors to balance it.

    ![Level Motors](../../assets/airframes/vtol/falcon_vertigo/falcon_vertigo_55_level_motors.jpg)

## 配置

Perform the normal [Basic Configuration](../config/index.md).

备注：

1. For [Airframe](../config/airframe.md) select the vehicle group/type as _Standard VTOL_ and the specific vehicle as [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol) as shown below.

  ![QCG - Select Generic Standard VTOL](../../assets/qgc/setup/airframe/px4_frame_generic_standard_vtol.png)

2. Set the [Autopilot Orientation](../config/flight_controller_orientation.md) to `ROTATION_YAW_270` as the autopilot is mounted [sideways](#flight_controller_orientation) with respect to the front of the vehicle. The compass is oriented forward, so you can leave that at the default (`ROTATION_NONE`).

3. Configure the outputs and geometry following the instructions in [Actuators Configuration](../config/actuators.md)

4. The default parameters are often sufficient for stable flight. For more detailed tuning information see [Standard VTOL Wiring and Configuration](../config_vtol/vtol_quad_configuration.md).

After you finish calibration the VTOL is ready to fly.

## 视频

<lite-youtube videoid="h7OHTigtU0s" title="PX4 Vtol test"/>

## 技术支持

If you have any questions regarding your VTOL conversion or configuration please visit <https://discuss.px4.io/c/px4/vtol>.
