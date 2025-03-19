# 위치 추정을 위한 비전 또는 모션 캡처 사용

전역 위치 소스를 사용할 수 없거나 신뢰할 수 없는 경우(예: 실내나 다리 아래를 비행시)에, VIO(Visual Inertial Odometry) 및 MoCap(모션 캡처) 시스템을 사용하여 차량 내비게이션이 가능합니다.

Both VIO and MoCap determine a vehicle's _pose_ (position and attitude) from "visual" information.
그들 사이의 주요 차이점은 프레임 관점입니다.

- VIO uses _onboard sensors_ to get pose data from the vehicle's perspective (see [egomotion](https://en.wikipedia.org/wiki/Visual_odometry#Egomotion)).
- MoCap uses a system of _off-board cameras_ to get vehicle pose data in a 3D space (i.e. it is an external system that tells the vehicle its pose).

두 시스템 유형의 포즈 데이터를 사용하여 PX4 자동조종장치의 로컬 위치 추정값(로컬 원점 기준)을 업데이트할 수 있으며, 선택적으로 차량 자세 추정을 융합할 수 있습니다. 또한 외부 포즈 시스템이 선형 속도 측정을 제공하는 경우에는 상태 추정 개선에 사용할 수 있습니다(선속도 측정의 융합은 EKF2에서만 지원됨).

This topic explains how to configure a PX4-based system to get data from MoCap/VIO systems (either via ROS or some other MAVLink system) and more specifically how to set up MoCap systems like VICON and Optitrack, and vision-based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) and [PTAM](https://github.com/ethz-asl/ethzasl_ptam)).

:::info
The instructions differ depending on whether you are using the EKF2 or LPE estimator.
:::

## PX4 MAVLink 통합

PX4 uses the following MAVLink messages for getting external position information, and maps them to [uORB topics](../middleware/uorb.md):

| MAVLink                                                                                                                                                                                                                                                | uORB                      |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                                                              | `vehicle_visual_odometry` |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_visual_odometry` |
| [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                                                                    | `vehicle_mocap_odometry`  |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED)) | `vehicle_mocap_odometry`  |

EKF2 only subscribes to `vehicle_visual_odometry` topics and can hence only process the first two messages
(a MoCap system must generate these messages to work with EKF2). 주행 거리 측정 메시지는 선형 속도도 PX4로 전송 가능한 유일한 메시지입니다.
LPE 추정기는 두 주제를 모두 구독하므로, 위의 모든 메시지를 처리할 수 있습니다.

:::tip
EKF2 is the default estimator used by PX4.
LPE보다 테스트 및 지원이 더 잘 되므로, 우선적으로 사용하여야 합니다.
:::

메시지는 30Hz(공분산을 포함하는 경우)와 50Hz 사이에서 스트리밍되어야 합니다.
메시지 비율이 너무 낮으면, EKF2가 외부 비전 메시지를 융합하지 않습니다.

The following MAVLink "vision" messages are not currently supported by PX4:
[GLOBAL_VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE),
[VISION_SPEED_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE),
[VICON_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE)

## 기준 프레임

PX4 uses FRD (X **F**orward, Y **R**ight and Z **D**own) for the local body frame as well for the reference frame. When using the heading of the magnetometer, the PX4 reference frame x axis will be aligned with north, so therefore it is called NED (X **N**orth, Y **E**ast, Z **D**own). 대부분의 경우 PX4 추정기의 기준 좌표계와 외부 포즈 추정 중 하나가 일치하지 않습니다.
Therefore the reference frame of the external pose estimate is named differently, it is called [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD).

기준 프레임의 소스에 따라 MAVLink Vision/MoCap 메시지를 보내기 전에, 포즈 추정값에 사용자 정의 변환을 적용하여야 합니다.
이것은 PX4 규칙에 맞도록 포즈 추정의 상위 및 하위 프레임 방향을 변경하는 데 필요합니다. Have a look at the MAVROS [_odom_ plugin](https://github.com/mavlink/mavros/blob/master/mavros_extras/src/plugins/odom.cpp) for the necessary transformations.

:::tip
ROS users can find more detailed instructions below in [Reference Frames and ROS](#reference-frames-and-ros).
:::

For example, if using the Optitrack framework the local frame has $x{}$ and $z{}$ on the horizontal plane (_x_ front and _z_ right) while _y_ axis is vertical and pointing up.
간단한 트릭은 NED 규칙을 얻기 위해 축을 변경하는 것입니다.

If `x_{mav}`, `y_{mav}` and `z_{mav}` are the coordinates that are sent through MAVLink as position feedback, then we obtain:

```
x_{mav} = x_{mocap}
y_{mav} = z_{mocap}
z_{mav} = - y_{mocap}
```

Regarding the orientation, keep the scalar part _w_ of the quaternion the same and swap the vector part _x_, _y_ and _z_ in the same way.
이 트릭을 모든 시스템에 적용할 수 있습니다. NED 프레임을 가져와야 하는 경우 MoCap 출력을 보고 그에 따라 축을 교체하십시오.

## EKF2 튜닝과 설정

참고: 간략한 개요입니다.
For more detailed information, check the [Using PX4's Navigation Filter (EKF2)](../advanced_config/tuning_the_ecl_ekf.md)

The following parameters must be set to use external position information with EKF2 (these can be set in _QGroundControl_ > **Vehicle Setup > Parameters > EKF2**).

| 매개변수                                                                                                                                                                                                                                                                                                                                                                                                                      | 외부 위치 추정 설정                                                                                                                                              |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [EKF2_EV_CTRL](../advanced_config/parameter_reference.md#EKF2_EV_CTRL)                                                                                                                                                                                                                                                                                                          | Set _horizontal position fusion_, _vertical vision fusion_, _velocity fusion_, and _yaw fusion_, according to your desired fusion model. |
| [EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF)                                                                                                                                                                                                                                                                                                          | Set to _Vision_ to use the vision as the reference source for altitude estimation.                                                       |
| [EKF2_EV_DELAY](../advanced_config/parameter_reference.md#EKF2_EV_DELAY)                                                                                                                                                                                                                                                                                                        | 측정 타임스탬프와 "실제" 캡처 시간 간의 차이로 설정합니다. For more information see [below](#tuning-EKF2_EV_DELAY).                              |
| [EKF2_EV_POS_X](../advanced_config/parameter_reference.md#EKF2_EV_POS_X), [EKF2_EV_POS_Y](../advanced_config/parameter_reference.md#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](../advanced_config/parameter_reference.md#EKF2_EV_POS_Z) | 로봇의 몸체 프레임을 기준으로 비전 센서(또는 MoCap 마커)의 위치를 설정합니다.                                                                       |

You can also disable GNSS, baro and range finder fusion using [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL), [EKF2_BARO_CTRL](../advanced_config/parameter_reference.md#EKF2_BARO_CTRL) and [EKF2_RNG_CTRL](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL), respectively.

:::tip
Reboot the flight controller in order for parameter changes to take effect.
:::

<a id="tuning-EKF2_EV_DELAY"></a>

#### EKF2_EV_DELAY 튜닝

[EKF2_EV_DELAY](../advanced_config/parameter_reference.md#EKF2_EV_DELAY) is the _Vision Position Estimator delay relative to IMU measurements_.

즉, 비전 시스템 타임스탬프와 IMU 시계(EKF2의 "기본 시계")에 의해 기록되었을 "실제" 캡처 시간 간의 차이입니다.

기술적으로, 이것은 MoCap과 (예를 들어) ROS 컴퓨터 사이에 정확한 타임스탬프 (도착 시간이 아님)와 시간 동기화 (예 : NTP)가있는 경우 0으로 설정할 수 있습니다.
In reality, this needs some empirical tuning since delays in the entire MoCap->PX4 chain are very setup-specific.
시스템이 완전히 동기화된 체인으로 설정되는 경우는 매우 드뭅니다.

IMU 속도와 EV 속도 간의 오프셋을 확인하여, 로그에서 대략적인 지연 추정치를 계산할 수 있습니다.
To enable logging of EV rates set bit 7 (Computer Vision and Avoidance) of [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE).

![ekf2_ev_delay log](../../assets/ekf2/ekf2_ev_delay_tuning.png)

:::info
A plot of external data vs. onboard estimate (as above) can be generated using [FlightPlot](../log/flight_log_analysis.md#flightplot) or similar flight analysis tools.
At time of writing (July 2021) neither [Flight Review](../log/flight_log_analysis.md#flight-review-online-tool) nor [MAVGCL](../log/flight_log_analysis.md#mavgcl) support this functionality.
:::

이 값은 동적 기동 중에 가장 낮은 EKF 혁신을 산출하는 값을 찾기 위하여, 매개변수를 변경하여 추가 튜닝할 수 있습니다.

## LPE 튜닝과 설정

You will first need to [switch to the LPE estimator](../advanced/switching_state_estimators.md) by setting the following parameters: [LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) (1), [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) (0), [ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) (0).

:::info
If targeting `px4_fmu-v2` hardware you will also need to use a firmware version that includes the LPE module (firmware for other FMU-series hardware includes both LPE and EKF).
The LPE version can be found in the zip file for each PX4 release or it can be built from source using the build command `make px4_fmu-v2_lpe`.
See [Building the Code](../dev_setup/building_px4.md) for more details.
:::

### 외부 포즈 입력 활성화

The following parameters must be set to use external position information with LPE (these can be set in _QGroundControl_ > **Vehicle Setup > Parameters > Local Position Estimator**).

| 매개변수                                                                                                                                    | 외부 위치 추정 설정                                                                                                                       |
| --------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| [LPE_FUSION](../advanced_config/parameter_reference.md#LPE_FUSION)                                                 | Vision integration is enabled if _fuse vision position_ is checked (it is enabled by default). |
| [ATT_EXT_HDG_M](../advanced_config/parameter_reference.md#ATT_EXT_HDG_M) | 외부 제목 통합을 활성화하려면 1 또는 2로 설정합니다. 1로 설정하면 비전이 사용되는 반면, 2로 설정하면 MoCap 제목 사용이 활성화됩니다.                 |

### 기압계 퓨전 비활성화

VIO 또는 MoCap 정보에서 이미 매우 정확한 고도를 사용할 수 있는 경우에는, LPE에서 기압 보정을 비활성화하여 Z축의 드리프트를 줄이는 것이 유용합니다.

This can be done by in _QGroundControl_ by unchecking the _fuse baro_ option in the [LPE_FUSION](../advanced_config/parameter_reference.md#LPE_FUSION) parameter.

### 노이즈 매개변수 조정

If your vision or MoCap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters: [LPE_VIS_XY](../advanced_config/parameter_reference.md#LPE_VIS_XY) and [LPE_VIS_Z](../advanced_config/parameter_reference.md#LPE_VIS_Z) (for VIO) or [LPE_VIC_P](../advanced_config/parameter_reference.md#LPE_VIC_P) (for MoCap).
표준변차 매개변수를 줄이면, 추정자가 들어오는 포즈 추정치를 더 신뢰하게 됩니다.
허용된 최소값보다 낮게 설정하고, 강제 저장해야 할 수도 있습니다.

:::tip
If performance is still poor, try increasing the [LPE_PN_V](../advanced_config/parameter_reference.md#LPE_PN_V) parameter.
이로 인해 추정자는 속도 추정 중에 측정값을 더 신뢰하게 됩니다.
:::

## Enabling Auto Modes with a Local Position

All PX4 automatic flight modes (such as [Mission](../flight_modes_mc/mission.md), [Return](../flight_modes_mc/return.md), [Land](../flight_modes_mc/land.md), [Hold](../flight_modes_mc/land.md), [Orbit](../flight_modes_mc/orbit.md))) require a _global_ position estimate, which would normally come from a GPS/GNSS system.

Systems that only have a _local_ position estimate (from MOCAP, VIO, or similar) can use the [SET_GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN) MAVLink message to set the origin of the EKF to a particular global location.
EKF will then provide a global position estimate based on origin and local frame position.

This can then be used when planning and executing indoor missions, or to set a local return point, and so on.

## ROS 연동

ROS is not _required_ for supplying external pose information, but is highly recommended as it already comes with good integrations with VIO and MoCap systems.
PX4는 위와 같이 설정되어 있어야 합니다.

### 포즈 데이터를 ROS로 가져오기

VIO와 MoCap 시스템은 포즈 데이터를 얻는 방법이 다르며, 자체 설정과 주제가 있습니다.

The setup for specific systems is covered [below](#setup_specific_systems).
다른 시스템의 경우에는 공급업체의 설정 문서를 참고하십시오.

<a id="relaying_pose_data_to_px4"></a>

### 포즈 데이터를 PX4로 중계

MAVROS에는 다음 파이프라인을 사용하여 VIO 또는 MoCap 시스템에서 시각적 추정을 릴레이하는 플러그인이 있습니다.

| ROS                                                                                       | MAVLink                                                                                                                                                                                                                                                | uORB                      |
| ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| /mavros/vision_pose/pose                                             | [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                                                              | `vehicle_visual_odometry` |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_visual_odometry` |
| /mavros/mocap/pose                                                                        | [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                                                                    | `vehicle_mocap_odometry`  |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)) | `vehicle_mocap_odometry`  |

위의 파이프라인 중 하나를 LPE와 함께 사용할 수 있습니다.

EKF2로 작업하는 경우 "비전" 파이프라인만 지원됩니다.
To use MoCap data with EKF2 you will have to [remap](http://wiki.ros.org/roslaunch/XML/remap) the pose topic that you get from MoCap:

- MoCap ROS topics of type `geometry_msgs/PoseStamped` or `geometry_msgs/PoseWithCovarianceStamped` must be remapped to `/mavros/vision_pose/pose`.
  The `geometry_msgs/PoseStamped` topic is most common as MoCap doesn't usually have associated covariances to the data.
- If you get data through a `nav_msgs/Odometry` ROS message then you will need to remap it to `/mavros/odometry/out`, making sure to update the `frame_id` and `child_frame_id` accordingly.
- The odometry frames `frame_id = odom`, `child_frame_id = base_link` can be changed by updating the file in `mavros/launch/px4_config.yaml`. However, the current version of mavros (`1.3.0`) needs to be able to use the tf tree to find a transform from `frame_id` to the hardcoded frame `odom_ned`. The same applies to the `child_frame_id`, which needs to be connected in the tf tree to the hardcoded frame `base_link_frd`. If you are using mavros `1.2.0` and you didn't update the file `mavros/launch/px4_config.yaml`, then you can safely use the odometry frames `frame_id = odom`, `child_frame_id = base_link` without much worry.
- Note that if you are sending odometry data to px4 using `child_frame_id = base_link`, then you need to make sure that the `twist` portion of the `nav_msgs/Odometry` message is **expressed in body frame**, **not in inertial frame!!!!!**.

### 기준 프레임과 ROS

ROS와 PX4에서 사용하는 로컬과 전역 프레임은 같지 않습니다.

| 프레임 | PX4                                                                 | ROS                                                                                                      |
| --- | ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| 몸체  | FRD (X **F**orward, Y **R**ight, Z **D**own)     | FLU (X **F**orward, Y **L**eft, Z **U**p), usually named `base_link`                  |
| 전역  | FRD or NED (X **N**orth, Y **E**ast, Z **D**own) | FLU or ENU (X **E**ast, Y **N**orth, Z **U**p), with the naming being `odom` or `map` |

:::tip
See [REP105: Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html) for more information about ROS frames.
:::

두 프레임 모두 아래 이미지에 표시됩니다(왼쪽의 FRD/오른쪽의 FLU).

![Reference frames](../../assets/lpe/ref_frames.png)

외부 방향 추정시 EKF2를 사용하면, 자북을 무시하거나 자북에 대한 방향 오프셋을 계산하고 보상할 수 있습니다. Depending on your choice the yaw angle is given with respect to either magnetic north or local _x_.

:::info
When creating the rigid body in the MoCap software, remember to first align the robot's local _x_ axis with the world _x_ axis otherwise the yaw estimate will have an offset. 이렇게 하면 외부 포즈 추정 융합이 제대로 작동하지 않을 수 있습니다.
본체와 기준 좌표계가 정렬될 때 요 각도는 0이어야 합니다.
:::

MAVROS를 사용하면 이 작업이 간단합니다.
ROS는 ENU 프레임을 관례로 사용하므로, ENU에서 위치 피드백을 제공하여야 합니다.
If you have an Optitrack system you can use [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) node which streams the object pose on a ROS topic already in ENU.
With a remapping you can directly publish it on `mocap_pose_estimate` as it is without any transformation and MAVROS will take care of NED conversions.

MAVROS 주행 거리 측정 플러그인을 사용하면, 좌표 프레임을 쉽게 처리할 수 있습니다.
ROS의 tf 패키지를 사용합니다. 외부 포즈 시스템에는 PX4와 일치하지 않는 완전히 다른 프레임 규칙이 있을 수 있습니다.
외부 포즈 추정의 바디 프레임은 MOCAP 소프트웨어에서 바디 프레임을 설정하는 방법이나 드론에 VIO 센서를 장착하는 방법에 따라 달라질 수 있습니다.
MAVROS 주행 거리 측정 플러그인은 MAVROS에 의해 알려진 기체의 FRD 또는 FLU 본체 프레임과 관련하여 외부 포즈의 자식 프레임이 어떻게 향하고 있는 지 알아야 합니다.
따라서 외부 포즈의 바디 프레임을 tf 트리에 추가하여야 합니다. 이것은 ROS 시작 파일에 다음 줄의 수정된 버전을 포함하여 수행할 수 있습니다.

```
  <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="0 0 0 <yaw> <pitch> <roll> base_link <external_pose_child_frame> 1000"/>
```

Make sure that you change the values of yaw, pitch and roll such that it properly attaches the external pose's body frame to the `base_link` or `base_link_frd`.
Have a look at the [tf package](http://wiki.ros.org/tf#static_transform_publisher) for further help on how to specify the transformation between the frames.
rviz를 사용하여 프레임을 올바르게 부착했는 지 확인할 수 있습니다. The name of the `external_pose_child_frame` has to match the child_frame_id of your `nav_msgs/Odometry` message.
외부 포즈의 기준 프레임에도 동일하게 적용됩니다. You have to attach the reference frame of the external pose as child to either the `odom` or `odom_frd` frame. 따라서, 다음 코드 줄을 적절하게 조정하십시오.

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

## 특정 시스템 설정

### OptiTrack MoCap

The following steps explain how to feed position estimates from an [OptiTrack](https://optitrack.com/motion-capture-robotics/) system to PX4.
MoCap 시스템이 보정된 것으로 가정합니다.
See [this video](https://www.youtube.com/watch?v=cNZaFEghTBU) for a tutorial on the calibration process.

#### Steps on the _Motive_ MoCap software

- Align your robot's forward direction with the [system +x-axis](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System)
- [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0). Give the robot a name that does not contain spaces, e.g. `robot1` instead of `Rigidbody 1`
- [Enable Frame Broadacst and VRPN streaming](https://www.youtube.com/watch?v=yYRNG58zPFo)
- 위쪽 축을 Z축으로 설정합니다(기본값은 Y).

#### 포즈 데이터를 ROS로 가져오기

- Install the `vrpn_client_ros` package
- 다음 명령어로 개별 주제에 대한 각 강체의 포즈를 얻을 수 있습니다.
  ```sh
  roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>
  ```

If you named the rigidbody as `robot1`, you will get a topic like `/vrpn_client_node/robot1/pose`

#### 포즈 데이터 중계/재매핑

MAVROS provides a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4.
Assuming that MAVROS is running, you just need to **remap** the pose topic that you get from MoCap `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`.
Note that there is also a `mocap` topic that MAVROS provides to feed `ATT_POS_MOCAP` to PX4, but it is not applicable for EKF2.
그러나 LPE에는 적용됩니다.

:::info
Remapping pose topics is covered above [Relaying pose data to PX4](#relaying_pose_data_to_px4) (`/vrpn_client_node/<rigid_body_name>/pose` is of type `geometry_msgs/PoseStamped`).
:::

위에서 설명한 대로 EKF2 매개변수를 설정하였으면, 이제 PX4가 설정되고 MoCap 데이터를 통합합니다.

이제 첫 번째 비행을 진행할 준비가 되었습니다.

## 첫 번째 비행

위에서 설명한 (특정) 시스템 중 하나를 설정하였으면, 이제 테스트할 준비가 되었습니다.
아래 지침은 MoCap 및 VIO 시스템에서 수행하는 방법을 설명합니다.

### 외부 추정 확인

첫 비행 전에 다음을 확인하십시오.

- Set the PX4 parameter `MAV_ODOM_LP` to 1.
  PX4 will then stream back the received external pose as MAVLink [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) messages.
- You can check these MAVLink messages with the _QGroundControl_ [MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html)
  In order to do this, yaw the vehicle until the quaternion of the `ODOMETRY` message is very close to a unit quaternion. (w=1, x=y=z=0)
- 이 시점에서 몸체 프레임은 외부 포즈 시스템의 기준 프레임과 정렬됩니다.
  차량을 구르거나 피칭하지 않고 단위 쿼터니언에 가까운 쿼터니언을 얻을 수 없다면, 프레임에 여전히 피치 또는 롤 오프셋이 있을 수 있습니다.
  이 경우에는 더 이상 진행하지 말고, 좌표 프레임을 다시 확인하십시오.
- 정렬되면 지면에서 차량을 들어올릴 수 있으며, 위치의 z 좌표가 감소하는 것을 볼 수 있습니다.
  차량을 앞쪽으로 움직이면, 위치의 x 좌표가 증가합니다.
  차량을 오른 쪽으로 이동하면, y 좌표는 증가합니다.
  외부 포즈 시스템에서 선형 속도도 전송하는 경우에는, 선형 속도를 확인하여야 합니다.
  Check that the linear velocities are in expressed in the _FRD_ body frame reference frame.
- Set the PX4 parameter `MAV_ODOM_LP` back to 0. PX4는 이 메시지의 스트리밍을 중지합니다.

이러한 단계가 유지되면, 첫 번째 비행을 시도할 수 있습니다.

로봇을 바닥에 놓고, MoCap 피드백 스트리밍을 시작합니다.
왼쪽(스로틀) 스틱을 내리고, 모터를 작동시킵니다.

이때 왼쪽 스틱을 가장 낮은 위치에 놓고, 위치 제어로 전환합니다.
초록불이 켜져야 합니다.
녹색 표시등은 위치 피드백을 사용할 수 있고, 위치 제어가 활성화되었음을 알려줍니다.

왼쪽 스틱을 가운데에 놓으면, 데드존입니다.
이 스틱 값으로 로봇은 고도를 유지합니다. 스틱을 올리면 기준 고도가 증가하고, 값을 낮추면 감소합니다.
x와 y의 오른쪽 스틱에 대해서도 동일합니다.

왼쪽 스틱의 값을 높이면 로봇이 이륙합니다. 바로 중간에 다시 놓습니다. 위치를 유지할 수 있는 지 확인하십시오.

If it works, you may want to set up an [offboard](offboard_control.md) experiment by sending position-setpoint from a remote ground station.
