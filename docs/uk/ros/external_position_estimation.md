# Використання Vision або Motion Capture систем для Position Estimation

Системи візуальної інерційної одометрії (VIO) та захоплення руху (MoCap) дозволяють транспортним засобам здійснювати навігацію, коли джерело глобального позиціонування недоступне або ненадійне (наприклад, в приміщенні або під час прольоту під мостом. тощо).

Both VIO and MoCap determine a vehicle's _pose_ (position and attitude) from "visual" information.
Основна відмінність між ними - перспектива кадру:

- VIO uses _onboard sensors_ to get pose data from the vehicle's perspective (see [egomotion](https://en.wikipedia.org/wiki/Visual_odometry#Egomotion)).
- MoCap uses a system of _off-board cameras_ to get vehicle pose data in a 3D space (i.e. it is an external system that tells the vehicle its pose).

Дані про положення, отримані від обох типів систем, можуть бути використані для оновлення оцінки локального положення автопілота на базі PX4 (відносно локальної точки відліку), а також, за бажанням, можуть бути інтегровані в оцінку положення транспортного засобу. Крім того, якщо зовнішня система позиціонування також забезпечує вимірювання лінійної швидкості, її можна використовувати для покращення оцінки стану (об'єднання вимірювань лінійної швидкості підтримується лише EKF2).

This topic explains how to configure a PX4-based system to get data from MoCap/VIO systems (either via ROS or some other MAVLink system) and more specifically how to set up MoCap systems like VICON and Optitrack, and vision-based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) and [PTAM](https://github.com/ethz-asl/ethzasl_ptam)).

:::info
The instructions differ depending on whether you are using the EKF2 or LPE estimator.
:::

## Інтеграція PX4 з MAVLink

PX4 uses the following MAVLink messages for getting external position information, and maps them to [uORB topics](../middleware/uorb.md):

| MAVLink                                                                                                                                                                                                                                                | uORB                      |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                                                              | `vehicle_visual_odometry` |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_visual_odometry` |
| [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                                                                    | `vehicle_mocap_odometry`  |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED)) | `vehicle_mocap_odometry`  |

EKF2 only subscribes to `vehicle_visual_odometry` topics and can hence only process the first two messages
(a MoCap system must generate these messages to work with EKF2). Повідомлення odometry є єдиним повідомленням, яке може також відправляти лінійні швидкості в PX4.
Оцінювач LPE підписується на обидві теми, тому може обробляти всі вищезазначені повідомлення.

:::tip
EKF2 is the default estimator used by PX4.
Він краще протестований і підтримується, ніж LPE, і його слід використовувати за умовчанням.
:::

Повідомлення повинні транслюватися з частотою між 30 Гц (якщо містять коваріанти) і 50 Гц.
Якщо частота повідомлень занадто низька, EKF2 не буде обробляти повідомлення з зовнішнього візуального спостереження.

The following MAVLink "vision" messages are not currently supported by PX4:
[GLOBAL_VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE),
[VISION_SPEED_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE),
[VICON_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE)

## Основні відмінності

PX4 uses FRD (X **F**orward, Y **R**ight and Z **D**own) for the local body frame as well for the reference frame. When using the heading of the magnetometer, the PX4 reference frame x axis will be aligned with north, so therefore it is called NED (X **N**orth, Y **E**ast, Z **D**own). Напрямок опорного каркасу оцінювача PX4 та напрямок зовнішньої оцінки положення в більшості випадків не збігаються.
Therefore the reference frame of the external pose estimate is named differently, it is called [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD).

Залежно від джерела вашого опорного каркасу, вам потрібно буде застосувати власне перетворення до оцінки положення перед надсиланням повідомлення MAVLink Vision/MoCap.
Це необхідно для зміни орієнтації батьківського та дочірнього каркасу оцінки положення так, щоб вона відповідала конвенції PX4. Have a look at the MAVROS [_odom_ plugin](https://github.com/mavlink/mavros/blob/master/mavros_extras/src/plugins/odom.cpp) for the necessary transformations.

:::tip
ROS users can find more detailed instructions below in [Reference Frames and ROS](#reference-frames-and-ros).
:::

For example, if using the Optitrack framework the local frame has $x{}$ and $z{}$ on the horizontal plane (_x_ front and _z_ right) while _y_ axis is vertical and pointing up.
Простий трюк полягає в тому, що обмінюються вісіми, щоб отримати конвенцію NED.

If `x_{mav}`, `y_{mav}` and `z_{mav}` are the coordinates that are sent through MAVLink as position feedback, then we obtain:

```
x_{mav} = x_{mocap}
y_{mav} = z_{mocap}
z_{mav} = - y_{mocap}
```

Regarding the orientation, keep the scalar part _w_ of the quaternion the same and swap the vector part _x_, _y_ and _z_ in the same way.
Ви можете застосувати цей трюк у будь-якій системі - якщо вам потрібно отримати рамку NED, подивіться на вивід вашого MoCap та обміняйте вісі відповідно.

## Конфігурація та налаштування EKF2

Примітка: це короткий огляд.
For more detailed information, check the [Using PX4's Navigation Filter (EKF2)](../advanced_config/tuning_the_ecl_ekf.md)

The following parameters must be set to use external position information with EKF2 (these can be set in _QGroundControl_ > **Vehicle Setup > Parameters > EKF2**).

| Параметр                                                                                                                                                                                                                                                                                                                                                                                                                  | Налаштування для Зовнішньої Оцінки Положення                                                                                                                             |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL)                                                                                                                                                                                                                                                                                                          | Set _horizontal position fusion_, _vertical vision fusion_, _velocity fusion_, and _yaw fusion_, according to your desired fusion model.                 |
| [EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF)                                                                                                                                                                                                                                                                                                          | Set to _Vision_ to use the vision as the reference source for altitude estimation.                                                                       |
| [EKF2_EV_DELAY](../advanced_config/parameter_reference.md#EKF2_EV_DELAY)                                                                                                                                                                                                                                                                                                        | Встановіть різницю між міткою часу вимірювання та "фактичним" часом захоплення. For more information see [below](#tuning-EKF2_EV_DELAY). |
| [EKF2_EV_POS_X](../advanced_config/parameter_reference.md#EKF2_EV_POS_X), [EKF2_EV_POS_Y](../advanced_config/parameter_reference.md#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](../advanced_config/parameter_reference.md#EKF2_EV_POS_Z) | Встановіть положення візійного датчика (або маркерів MoCap) відносно тіла робота.                                                     |

You can also disable GNSS, baro and range finder fusion using [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL), [EKF2_BARO_CTRL](../advanced_config/parameter_reference.md#EKF2_BARO_CTRL) and [EKF2_RNG_CTRL](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL), respectively.

:::tip
Reboot the flight controller in order for parameter changes to take effect.
:::

<a id="tuning-EKF2_EV_DELAY"></a>

#### Налаштування EKF2_EV_DELAY

[EKF2_EV_DELAY](../advanced_config/parameter_reference.md#EKF2_EV_DELAY) is the _Vision Position Estimator delay relative to IMU measurements_.

Іншими словами, це різниця між міткою часу системи комп'ютерного зору та "фактичним" часом захоплення, який був би зафіксований годинником IMU ("базовим годинником" для EKF2).

Технічно цей параметр можна встановити на 0, якщо між комп'ютерами MoCap і (наприклад) ROS є правильне маркування часу (не тільки час прибуття) і синхронізація часу (наприклад, NTP).
In reality, this needs some empirical tuning since delays in the entire MoCap->PX4 chain are very setup-specific.
Рідко коли система налаштована з повністю синхронізованим ланцюжком!

Приблизну оцінку затримки можна отримати з логів, перевіривши зсув між частотами IMU та EV.
To enable logging of EV rates set bit 7 (Computer Vision and Avoidance) of [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE).

![ekf2_ev_delay log](../../assets/ekf2/ekf2_ev_delay_tuning.png)

:::info
A plot of external data vs. onboard estimate (as above) can be generated using [FlightPlot](../log/flight_log_analysis.md#flightplot) or similar flight analysis tools.
At time of writing (July 2021) neither [Flight Review](../log/flight_log_analysis.md#flight-review-online-tool) nor [MAVGCL](../log/flight_log_analysis.md#mavgcl) support this functionality.
:::

Значення можна додатково налаштувати, змінюючи параметр, щоб знайти значення, яке дає найнижчі інновації EKF під час динамічних маневрів.

## LPE Конфігурація/Налаштування

You will first need to [switch to the LPE estimator](../advanced/switching_state_estimators.md) by setting the following parameters: [LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) (1), [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) (0), [ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) (0).

:::info
If targeting `px4_fmu-v2` hardware you will also need to use a firmware version that includes the LPE module (firmware for other FMU-series hardware includes both LPE and EKF).
The LPE version can be found in the zip file for each PX4 release or it can be built from source using the build command `make px4_fmu-v2_lpe`.
See [Building the Code](../dev_setup/building_px4.md) for more details.
:::

### Увімкнення зовнішнього введення позиції

The following parameters must be set to use external position information with LPE (these can be set in _QGroundControl_ > **Vehicle Setup > Parameters > Local Position Estimator**).

| Параметр                                                                                                                                | Налаштування для Зовнішньої Оцінки Положення                                                                                                                                                                        |
| --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [LPE_FUSION](../advanced_config/parameter_reference.md#LPE_FUSION)                                                 | Vision integration is enabled if _fuse vision position_ is checked (it is enabled by default).                                                                                   |
| [ATT_EXT_HDG_M](../advanced_config/parameter_reference.md#ATT_EXT_HDG_M) | Встановіть значення 1 або 2, щоб увімкнути інтеграцію зовнішніх заголовків. Встановлення значення 1 призведе до використання зору, тоді як 2 увімкне використання заголовків MoCap. |

### Вимкнення Barometer Fusion

Якщо високоточна висота вже доступна з інформації VIO або MoCap, може бути корисно вимкнути корекцію баро в LPE, щоб зменшити дрейф по осі Z.

This can be done by in _QGroundControl_ by unchecking the _fuse baro_ option in the [LPE_FUSION](../advanced_config/parameter_reference.md#LPE_FUSION) parameter.

### Параметри налаштування шуму

If your vision or MoCap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters: [LPE_VIS_XY](../advanced_config/parameter_reference.md#LPE_VIS_XY) and [LPE_VIS_Z](../advanced_config/parameter_reference.md#LPE_VIS_Z) (for VIO) or [LPE_VIC_P](../advanced_config/parameter_reference.md#LPE_VIC_P) (for MoCap).
Зменшення їх призведе до того, що оцінювач буде більше довіряти вхідній оцінці положення.
Можливо, вам доведеться встановити їх нижче допустимого мінімуму та ввімкнути примусове збереження.

:::tip
If performance is still poor, try increasing the [LPE_PN_V](../advanced_config/parameter_reference.md#LPE_PN_V) parameter.
Це змусить оцінювача більше довіряти вимірюванням під час оцінювання швидкості.
:::

## Увімкнення автоматичних режимів з локальним розташуванням

All PX4 automatic flight modes (such as [Mission](../flight_modes_mc/mission.md), [Return](../flight_modes_mc/return.md), [Land](../flight_modes_mc/land.md), [Hold](../flight_modes_mc/land.md), [Orbit](../flight_modes_mc/orbit.md))) require a _global_ position estimate, which would normally come from a GPS/GNSS system.

Systems that only have a _local_ position estimate (from MOCAP, VIO, or similar) can use the [SET_GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN) MAVLink message to set the origin of the EKF to a particular global location.
Після цього EKF надасть оцінку глобального положення на основі походження та локального положення у просторі.

Це може бути використано при плануванні та виконанні місій у приміщенні, для встановлення місцевої точки повернення тощо.

## Робота з ROS

ROS is not _required_ for supplying external pose information, but is highly recommended as it already comes with good integrations with VIO and MoCap systems.
PX4 вже мають бути налаштовані як вище.

### Отримання даних про позицію в ROS

Системи VIO та MoCap мають різні способи отримання даних про положення, а також власні налаштування та теми.

The setup for specific systems is covered [below](#setup_specific_systems).
Для інших систем зверніться до документації з налаштування виробника.

<a id="relaying_pose_data_to_px4"></a>

### Передача даних про позицію до PX4

MAVROS має плагіни для передачі візуальної оцінки з системи VIO або MoCap за допомогою наступних пайплайнів:

| ROS                                                                                       | MAVLink                                                                                                                                                                                                                                                | uORB                      |
| ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| /mavros/vision_pose/pose                                             | [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                                                              | `vehicle_visual_odometry` |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_visual_odometry` |
| /mavros/mocap/pose                                                                        | [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                                                                    | `vehicle_mocap_odometry`  |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_mocap_odometry`  |

Ви можете використовувати будь-який з наведених вище пайплайнів за допомогою LPE.

Якщо ви працюєте з EKF2, підтримуються лише "vision" пайплайни.
To use MoCap data with EKF2 you will have to [remap](http://wiki.ros.org/roslaunch/XML/remap) the pose topic that you get from MoCap:

- MoCap ROS topics of type `geometry_msgs/PoseStamped` or `geometry_msgs/PoseWithCovarianceStamped` must be remapped to `/mavros/vision_pose/pose`.
  The `geometry_msgs/PoseStamped` topic is most common as MoCap doesn't usually have associated covariances to the data.
- If you get data through a `nav_msgs/Odometry` ROS message then you will need to remap it to `/mavros/odometry/out`, making sure to update the `frame_id` and `child_frame_id` accordingly.
- The odometry frames `frame_id = odom`, `child_frame_id = base_link` can be changed by updating the file in `mavros/launch/px4_config.yaml`. However, the current version of mavros (`1.3.0`) needs to be able to use the tf tree to find a transform from `frame_id` to the hardcoded frame `odom_ned`. The same applies to the `child_frame_id`, which needs to be connected in the tf tree to the hardcoded frame `base_link_frd`. If you are using mavros `1.2.0` and you didn't update the file `mavros/launch/px4_config.yaml`, then you can safely use the odometry frames `frame_id = odom`, `child_frame_id = base_link` without much worry.
- Note that if you are sending odometry data to px4 using `child_frame_id = base_link`, then you need to make sure that the `twist` portion of the `nav_msgs/Odometry` message is **expressed in body frame**, **not in inertial frame!!!!!**.

### Референсні системи координат та ROS

Локальна/світова та світова системи координат, що використовуються в ROS та PX4, відрізняються.

| Frame | PX4                                                                 | ROS                                                                                                      |
| ----- | ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| Body  | FRD (X **F**orward, Y **R**ight, Z **D**own)     | FLU (X **F**orward, Y **L**eft, Z **U**p), usually named `base_link`                  |
| World | FRD or NED (X **N**orth, Y **E**ast, Z **D**own) | FLU or ENU (X **E**ast, Y **N**orth, Z **U**p), with the naming being `odom` or `map` |

:::tip
See [REP105: Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html) for more information about ROS frames.
:::

Обидві системи координат показані на зображенні нижче (FRD зліва / FLU справа).

![Reference frames](../../assets/lpe/ref_frames.png)

З EKF2 при використанні зовнішньої оцінки напрямку магнітного північ може бути або ігноруватися, або зміщення напрямку до магнітного північного може бути розраховано та скомпенсовано. Depending on your choice the yaw angle is given with respect to either magnetic north or local _x_.

:::info
When creating the rigid body in the MoCap software, remember to first align the robot's local _x_ axis with the world _x_ axis otherwise the yaw estimate will have an offset. Це може призупинити правильну роботу злиття зовнішньої оцінки позиції.
Кут крену повинен дорівнювати нулю, коли тіло та опорна система вирівнюються.
:::

Використовуючи MAVROS, ця операція є простою.
ROS використовує фрейми ENU як конвенцію, тому зворотний зв'язок щодо позиції повинен бути наданий в ENU.
If you have an Optitrack system you can use [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) node which streams the object pose on a ROS topic already in ENU.
With a remapping you can directly publish it on `mocap_pose_estimate` as it is without any transformation and MAVROS will take care of NED conversions.

Плагін MAVROS відомий тим, що спрощує роботу з координатними рамками.
Він використовує пакет tf ROS. Ваш зовнішній система позиціонування може мати зовсім іншу конвенцію рамки, яка не відповідає конвенції PX4.
Корпусна рама зовнішньої оцінки позиції може залежати від того, як ви встановите корпусну раму в програмному забезпеченні MOCAP або від того, як ви встановите сенсор VIO на дрона.
Плагін відомостей MAVROS потребує знання, як дитяча рамка зовнішньої позиції орієнтована відносно рамки тіла FRD або FLU повітряного судна, відомої за допомогою MAVROS.
Отже, вам потрібно додати зовнішню позу тіла до дерева tf. Це можна зробити, включивши адаптовану версію наступного рядка до вашого ROS-файлу запуску.

```
  <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="0 0 0 <yaw> <pitch> <roll> base_link <external_pose_child_frame> 1000"/>
```

Make sure that you change the values of yaw, pitch and roll such that it properly attaches the external pose's body frame to the `base_link` or `base_link_frd`.
Have a look at the [tf package](http://wiki.ros.org/tf#static_transform_publisher) for further help on how to specify the transformation between the frames.
Ви можете використовувати rviz, щоб перевірити, чи ви правильно прикріпили рамку. The name of the `external_pose_child_frame` has to match the child_frame_id of your `nav_msgs/Odometry` message.
Те ж саме стосується і для опорної рамки зовнішньої позиції. You have to attach the reference frame of the external pose as child to either the `odom` or `odom_frd` frame. Адаптуйте тому відповідно кодовий рядок.

```
  <node pkg="tf" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
        args="0 0 0 <yaw> <pitch> <roll> odom <external_pose_parent_frame> 1000"/>
```

If the reference frame has the z axis pointing upwards you can attached it without any rotation (yaw=0, pitch=0, roll=0) to the `odom` frame.
The name of `external_pose_parent_frame` has to match the frame_id of the odometry message.

:::info
When using the MAVROS _odom_ plugin, it is important that no other node is publishing a transform between the external pose's reference and child frame.
This might break the _tf_ tree.
:::

<a id="setup_specific_systems"></a>

## Конкретні налаштування системи

### OptiTrack MoCap

The following steps explain how to feed position estimates from an [OptiTrack](https://optitrack.com/motion-capture-robotics/) system to PX4.
Припускається, що система MoCap налаштована.
See [this video](https://www.youtube.com/watch?v=cNZaFEghTBU) for a tutorial on the calibration process.

#### Steps on the _Motive_ MoCap software

- Align your robot's forward direction with the [system +x-axis](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System)
- [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0). Give the robot a name that does not contain spaces, e.g. `robot1` instead of `Rigidbody 1`
- [Enable Frame Broadacst and VRPN streaming](https://www.youtube.com/watch?v=yYRNG58zPFo)
- Встановіть вісь Up на ось Z (за замовчуванням - Y)

#### Отримання даних про позицію в ROS

- Install the `vrpn_client_ros` package
- Ви можете отримати позу кожного жорсткого тіла на окрему тему, запустивши
  ```sh
  roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>
  ```

If you named the rigidbody as `robot1`, you will get a topic like `/vrpn_client_node/robot1/pose`

#### Передача / перенаправлення даних про позу

MAVROS provides a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4.
Assuming that MAVROS is running, you just need to **remap** the pose topic that you get from MoCap `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`.
Note that there is also a `mocap` topic that MAVROS provides to feed `ATT_POS_MOCAP` to PX4, but it is not applicable for EKF2.
Однак, це застосовується з LPE.

:::info
Remapping pose topics is covered above [Relaying pose data to PX4](#relaying_pose_data_to_px4) (`/vrpn_client_node/<rigid_body_name>/pose` is of type `geometry_msgs/PoseStamped`).
:::

Припускаючи, що ви налаштували параметри EKF2, як описано вище, PX4 тепер встановлений і об'єднує дані MoCap.

Ви тепер готові перейти до першого політ.

## Перший політ

Після налаштування однієї з (специфічних) систем, описаних вище, ви повинні бути готові до тесту.
Інструкції нижче показують, як це зробити для систем MoCap та VIO

### Перевірте зовнішню оцінку

Перед першим польотом обов'язково виконайте наступні перевірки:

- Set the PX4 parameter `MAV_ODOM_LP` to 1.
  PX4 will then stream back the received external pose as MAVLink [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) messages.
- You can check these MAVLink messages with the _QGroundControl_ [MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html)
  In order to do this, yaw the vehicle until the quaternion of the `ODOMETRY` message is very close to a unit quaternion. (w=1, x=y=z=0)
- На цьому етапі корпус виробу зорієнтований у відповідності з ориєнтацією відносно зовнішньої системи координат.
  Якщо вам не вдається отримати кватерніон, близький до одиничного, без обертання або нахилу вашого літака, це, ймовірно, означає, що ваша рама все ще має зміщення нахилу або кочування.
  У цьому випадку не продовжуйте і перевірте знову свої координатні рамки.
- Після зорієнтування ви можете підняти літак з землі, і ви маєте бачити, як координата z позиції зменшується.
  Переміщення засобу в напрямку вперед повинно збільшити координату X.
  Під час руху транспортного засобу вправо слід збільшувати координату y.
  У разі, якщо ви також надсилаєте лінійні швидкості зовнішньої системи позиціонування, вам також слід перевірити лінійні швидкості.
  Check that the linear velocities are in expressed in the _FRD_ body frame reference frame.
- Set the PX4 parameter `MAV_ODOM_LP` back to 0. PX4 припинить передавати це повідомлення назад.

Якщо ці кроки є послідовними, ви можете спробувати свій перший польот.

Покладіть робота на землю і почніть передавати зворотний зв'язок MoCap.
Потягніть палицю газу вниз і зберметизуйте двигуни.

На цьому етапі, зліва палиця на найнижчому положенні, перейдіть у режим позиціонного контролю.
Ви повинні побачити зелену лампочку.
Зелена лампочка свідчить про те, що доступний зворотний зв'язок позиції, і позиційний контроль активований.

Помістіть лівий джойстик в середину, це зона мертвої зони.
З цим значенням палиці робот підтримує свою висоту;
підняття палиці збільшить висоту посилки, тоді як зниження значення зменшить її.
Те ж саме для правої палиці по x та y.

Збільште значення лівої палиці, і робот злетить,
поверніть його назад у середину праворуч після цього. Перевірте, чи він може утримати своє положення.

If it works, you may want to set up an [offboard](offboard_control.md) experiment by sending position-setpoint from a remote ground station.
