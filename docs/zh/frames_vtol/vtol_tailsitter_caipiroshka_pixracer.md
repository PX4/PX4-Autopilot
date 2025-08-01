# TBS Caipiroshka

The Caipiroshka VTOL is a slightly modified _TBS Caipirinha_.

:::info
The _TBS Caipirinha_ has been superseded and is no longer available.
These instructions _should_ work with the updated vehicle: [TBS Caipirinha 2](https://team-blacksheep.com/products/prod:tbs_caipi2_pnp).
A number of other components have been updated in the parts list too.
:::

<lite-youtube videoid="acG0aTuf3f8" title="PX4 VTOL - Call for Testpilots"/>

## 配件列表

- TBS Caipirinha Wing (no longer available - try [TBS Caipirinha 2](https://team-blacksheep.com/products/prod:tbs_caipi2_pnp))
- Left and right 3D-printed motor mount (<a href="https://github.com/PX4/PX4-Autopilot/raw/main/docs/assets/airframes/vtol/caipiroshka/motor_mounts.zip" target="_blank">design files</a>)
- CW 8045 propeller ([Eflight store](https://www.banggood.com/GEMFAN-Carbon-Nylon-8045-CWCCW-Propeller-For-Quadcopters-1-Pair-p-950874.html))
- CCW 8045 propeller ([Eflight store](https://www.banggood.com/GEMFAN-Carbon-Nylon-8045-CWCCW-Propeller-For-Quadcopters-1-Pair-p-950874.html))
- 2 x 1800kv 120-180W电机
  - [ePower 2208](https://www.galaxus.ch/en/s5/product/epower-22081400-fuer-2-3-lipo-imax-rc-motors-8355913)
  - [Armattan 2208 1800kV Multirotor Motor](https://www.amazon.com/Armattan-2208-1800kV-Multirotor-Motor/dp/B00UWLW0C8)
    <!-- equivalent replacement must match: kV (1800), motor size (2208) and number of LiPo cells (3S). -->
- 2 x 20-30S 电调
  - [GetFPV](https://www.getfpv.com/lumenier-30a-blheli-s-esc-opto-2-4s.html)
- BEC（3A，5-5.3V）（如果你的电调不能提供5V的输出，可以用这个）
- 3S 2200 mA锂电池
  - Team Orion 3S 11.1V 50 C ([Hobbyshop store](https://www.hobbyshop.ch/modellbau-elektronik/akku/team-orion-lipo-2200-3s-11-1v-50c-xt60-ori60163.html))
- [Pixracer autopilot board + power module](../flight_controller/pixracer.md)
- [Digital airspeed sensor](https://hobbyking.com/en_us/hkpilot-32-digital-air-speed-sensor-and-pitot-tube-set.html)

## 组装

The picture below shows what a fully assembled Caipiroshka could look like.

![Caipiroshka](../../assets/airframes/vtol/caipiroshka/caipiroshka.jpg)

In the following some general tips on how to build the vehicle will be given.

### 飞控

Mount the autopilot in the middle close to the CG of the airframe.

### 安装电机

Print the motor mount (2 times) of which the link to the STL file was specified in the part list.
Attach one motor mount on each wing side such that the motor axis will be roughly going through the center of the elevons (see picture).
In the upper picture the horizontal distance between the two motor mounts is 56cm.
Once you have marked the correct position on the wing you can cover the area which will be in contact with the mount with standard transparent tape on both the upper and lower wing side.
Then apply a layer of hot glue onto this area and glue the motor mount onto the wing.
The reason for having tape in between the wing surface and the hot glue is that you can very easily remove the motor mount by ripping of the tape from the wing without any damage.
This is useful when trying to replace a damaged motor mount.

### 电调:

The motor controllers can be directly mounted on a flat surface of the motor mounts using glue or a cable binder.
You will have to route the power cables to the battery bay. You can use an old soldering iron to melt channels into the foam.
Connect the power cables of both motor controllers in the battery bay and solder a plug to the end.
This will enable you to connect both the motor controllers to the power module.
If you don't have motor controllers which can provide 5V for the output rail of the autopilot then you will have to use an external power supply (BEC).

### GPS

The GPS can be mounted in the middle at the very back of the airframe. This helps shifting the weight of the plane to the back since the two motors, a camera and a potentially bigger battery can make it quite nose heavy.
Also the large distance to the 12V power cables is beneficial for reducing magnetic interference of the external magnetometer.

### 空速传感器

Attach the pitot tube close to the outside edge of one of the wing sides.
Make sure that the pitot is not affected by the airflow of the propeller.
You should be fine if the horizontal distance from the tube to the axis of the motors is larger than the radius of the propeller.
Use e.g. an old soldering iron to create a recess for the pitot tube, the tubing and the actual sensor (see picture).
Create a channel for routing the cable across the wing to the other components.

### 连接传感器到 I2C 接口

Both the airspeed sensor and the external magnetometer (located in the gps housing) need to be connected to the I2C bus of the autopilot.
Therefore, you will have to use an I2C splitter like the one indicated in the part list.
Connect the splitter board with the I2C bus of the autopilot.
Then connect both the external magnetometer and the airspeed sensor to the splitter board with a standard I2C cable.
In the upper picture the splitter board is located on the left side of the GPS unit.

### 升降副翼

The elevons can be attached to the back side of the wing using transparent tape. You can follow the instructions provided by Team Blacksheep in the build manual for the TBS Caiprinha airframe.

### 普通安装规则

Before mounting all the components to the wing, use tape to hold them in the approximate position and check if the CG of the wing is in the recommended range specified in the build manual for the TBS Caipirinha.
Depending on the additional components you want to have onboard (e.g. GoPro in front or bigger battery) you will need to shift the location of components.

## 机架设置

Switch to the configuration section in [QGroundControl](../config/airframe.md) and select the airframe tab.
Scroll down the list to find the _VTOL Duo Tailsitter_ icon. Select the _Caipiroshka Duo Tailsitter_ from the drop-down list.

![caipiroshka](../../assets/airframes/vtol/caipiroshka/airframe_px4_vtol_caipiroshka_duo_tailsitter.jpg)

## 连接舵机

The descriptions in the table below are referring to the user facing the front of the vehicle when it lies flat on its belly on a table.

| 输出    | 频率     | 执行器  |
| ----- | ------ | ---- |
| MAIN1 | 400 Hz | 右侧电机 |
| MAIN2 | 400 Hz | 左侧电机 |
| MAIN3 | 400 Hz | 空    |
| MAIN4 | 400 Hz | 空    |
| MAIN5 | 50 Hz  | 右副翼  |
| MAIN6 | 50 Hz  | 左副翼  |
