# 安装GPS/指南针

Compass and GNSS/Compass modules should be mounted on the frame as far away from motor/ESC power lines and other sources of electromagnetic interference as possible, and [oriented](#compass-orientation) upright with the direction marker pointing towards the front of the vehicle.
You should also configure PX4 to [set the position](#position) of the receiver relative to the centre-of-gravity (CoG).

On multicopters it is common to mount the compass on a pedestal, while for fixed-wing and VTOL vehicles the compass is usually mounted on a wing.

## Compass Orientation

The compass should ideally be oriented so that it is upright and the direction marker is pointing towards the front of the vehicle (the default orientation), but if needed can be oriented at multiples of 45° from this attitude (in any axis) as defined in the [standard MAVLink orientations](https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION) (these follow the same frame convention as when [orienting the flight controller](../config/flight_controller_orientation.md#calculating-orientation)).

The diagram below shows the heading marker on the Pixhawk 4 flight controller and compass.

![Connect compass/GPS to Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_compass_gps.jpg)

PX4 will automatically detect the orientation for any of these standard orientations during [compass calibration](../config/compass.md) ([by default](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT)).

The compass can also be mounted at any other "custom euler angles", but in this case you will need to manually configure the orientations.
For more information see [Setting the Compass Orientation](../config/flight_controller_orientation.md#setting-the-compass-orientation) in _Flight Controller/Sensor Orientation_.

## 安装位置

In order to compensate for the relative motion between the receiver and the CoG, you should [configure](../advanced_config/parameters.md) the following parameters to set the offsets: [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z).

This is important because the body frame estimated by the EKF will converge on the location of the GNSS module and assume it to be at the CoG. If the GNSS module is significantly offset from the CoG, then rotation around the COG will be interpreted as an altitude change, which in some flight modes (such as position mode) will result in unnecessary corrections.

It is particularly important if using [RTK GNSS](../advanced/rtk_gps.md) which has centimeter-level accuracy, because if the offsets are not set then GNSS measurements will often be rejected as inconsistent with the current EFK estimate.

:::details
Explanation
For example, if the GNSS module is 10cm above the CoG, and the IMU is located at the GoG, a pitch motion of 1 rad/s will create a GNSS velocity measurement of 10cm/s _even though the CoG isn't moving_.
If the speed accuracy of the GNSS receiver is 1cm/s, the EKF might stop trusting the measurements because they appear inconsistent (wrong by 10x the accuracy).
If the offsets are defined, the EKF will correct the measurements using the gyro data.
:::
