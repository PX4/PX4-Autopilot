# Vehicle (Frame) Selection

After installing firmware you need to select a [vehicle type and frame configuration](../airframes/airframe_reference.md).
This applies appropriate initial parameter values for the selected frame, such as the vehicle type, number of motors, relative motor position, and so on.
These can later be customised for your vehicle in [Actuator Configuration & Testing](../config/actuators.md).

::: tip
Choose the frame that matches your vehicle brand and model if one exists, and otherwise select the closest "Generic" frame option matching your vehicle.
:::

## Set the Frame

To set the airframe:

1. Start _QGroundControl_ and connect the vehicle.
1. Select **"Q" icon > Vehicle Setup > Airframe** (sidebar) to open _Airframe Setup_.
1. Select the broad vehicle group/type that matches your airframe and then use the dropdown within the group to choose the airframe that best matches your vehicle.

   ![Selecting generic hexarotor X frame in QGroundControl](../../assets/qgc/setup/airframe/airframe_px4.jpg)

   The example above shows _Generic Hexarotor X geometry_ selected from the _Hexarotor X_ group.

1. Click **Apply and Restart**.
   Click **Apply** in the following prompt to save the settings and restart the vehicle.

   <img src="../../assets/qgc/setup/airframe/airframe_px4_apply_prompt.jpg" width="300px" title="Apply airframe selection prompt" />

## Next Steps

[Actuator Configuration & Testing](../config/actuators.md) shows how to set the precise geometry of the vehicle motors and actuators, and their mapping to flight controller outputs.
After mapping actuators to outputs you should perform [ESC Calibration](../advanced_config/esc_calibration.md) if using PWM or OneShot ESCs.

## Further Information

- [QGroundControl User Guide > Airframe](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/airframe.html)
- [PX4 Setup Video - @37s](https://youtu.be/91VGmdSlbo4?t=35s) (Youtube)
