# PX4 ROS 2 控制接口

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
在撰写本文时，PX4 ROS 2 控制接口的部分内容仍处于实验阶段，因此可能会发生变动：

- ROS 2 模式中用于定义模式的架构及核心接口在很大程度上已趋于稳定，且会在CI中进行测试
  相比当前状态下的离板模式，该代码库具有显著优势
- 仅有少数几种设定点类型已趋于稳定。（其余设定点类型仍在开发中）
  您可能需要使用内部的 PX4 主题，这些主题可能不会随着时间的推移而保持与后方兼容。
- 该 API没有完整的文档

:::

这[PX4 ROS 2接口库](../ros2/px4_ros2_interface_lib.md)是一个 C++ 库，可简化从 ROS 2 控制 PX4 的操作。

开发者可使用该库创建并动态注册以 ROS 2 编写的模式。
这些模式会动态注册到 PX4 中，并且对于地面站或其他外部系统而言，它们看起来就像是 PX4 的一部分。
开发者甚至可以用功能增强的 ROS 2 版本模式替换 PX4 中的默认模式，且若 ROS 2 模式失效，系统会回退到原始版本的模式。

该库还提供了用于发送不同类型设定点的类，其涵盖范围从高层级的导航任务一直到直接的执行器控制。
这些类对 PX4 所使用的内部设定点进行了抽象处理，因此可用于为未来的 PX4 和 ROS 版本提供统一的 ROS 2 接口。

PX4 ROS 2 模式相较于 PX4 内部模式，更易于实现和维护，并且在处理能力与既有代码库资源方面，能为开发者提供更丰富的支持。
除非该模式属于安全关键型、对时序有严格要求或需要极高的更新速率，或者你的飞行器没有搭载伴随计算机，否则你应优先[考虑使用 PX4 ROS 2 模式，而非 PX4 内部模式](../concept/flight_modes.md#internal-vs-external-modes)。

## 综述

该图从概念层面概述了控制接口模式与模式执行器如何与 PX4 进行交互。

![ROS2 模式概览图](../../assets/middleware/ros2/px4_ros2_interface_lib/ros2_modes_overview.svg)

<!-- Source: https://docs.google.com/drawings/d/1WByCfgcytnaow7r41VhYJL8OGrw1RjFO51GoPMQBCNA/edit -->

以下章节对图表中使用的术语进行定义和解释。

### 定义

#### 模式

使用接口库定义的模式具有以下特性：

- 模式是一种组件，它可以向载具发送设定值以控制其运动（例如速度指令或直接执行器指令）。
- 模式在激活状态下会选择一种设定值类型并发送该设定值。
  它能够在多种设定值类型之间进行切换。
- 一种模式无法激活其他模式，其必须由以下对象激活：用户（通过RC/GCS）、处于故障保护状态下的飞控、mode executor_，或其他某种外部系统。
- 具有一个由GCS显示的名称。
- 可配置其模式要求（例如，要求具备有效的位置估算值）
- 一种模式可执行不同任务，例如飞往目标点、下放绞盘、释放有效载荷，之后返航。
- 一种模式可替换 PX4 中已定义的模式。

#### 模式执行器

模式执行器是用于调度模式的可选组件。
例如，用于自定义有效载荷投放或测绘模式的模式执行器，可能会先触发起飞，随后切换至该自定义模式，待模式完成后再触发RTL。

具体而言，它具有以下特性：

- 模式执行器是一种可选组件，其层级比模式高一级。
  它是一种状态机，能够激活模式并等待模式完成。
- 它仅能在负责状态下执行此操作。
  为此，一个执行器恰好拥有一个_owned mode_(且一个模式最多只能被一个执行器拥有)。
  该模式可作为执行器的激活触发条件：当用户选择此模式时，其所属的执行器会被激活，进而能够选择任意模式。
  它会一直处于管控状态，直至用户（通过遥控器或从地面控制站）切换模式，或故障保护机制触发模式切换。
  若故障保护状态解除，执行器将重新激活。
- 这允许多个执行器共存。
- 执行器无法激活其他执行器。
- 在该库中，模式执行器始终需结合自定义模式来实现。

::: info

- 这些定义确保用户可在任意时刻通过RC或GCS发送模式切换指令，从而夺回对自定义模式或执行器的控制权。
- 模式执行器对用户而言是透明的。
  它通过所属模式被间接选择并激活，因此该模式应相应地命名。

:::

#### 配置覆盖

模式和执行器均可定义配置覆盖，从而能在模式或执行器处于激活状态时，对特定行为进行自定义设置。

这些措施目前正在实施：

- _Disabling auto-disarm_.
  这允许着陆，然后再次起飞(例如释放有效载荷)。
- _Ability to defer non-essential failsafes_.
  这使得执行器能够执行某项操作，且不会被非关键故障保护中断。
  例如，忽略低电量故障保护（机制），以便绞车操作能够完成。

### 与离板控制的比较

上述概念提供了一些优点，而不是传统的[离板控制](../ros/offboard_control.md)：

- 多个节点或应用程序可以共存，甚至能同时运行。
  但在特定时刻，仅能有一个节点对载具进行控制，且该节点的定义是明确的。
- 模式具有独特的名称，并且可在GCS中显示 / 选择。
- 模式与故障保护状态机及解锁检查功能相集成。
- 可发送的设定值类型定义明确。
- ROS 2 模式可以替换飞行控制器内部模式(如[返回模式](../flight_modes/return.md))。

## 安装与首次测试

开始使用前，需完成以下步骤：

1. 请确保您在 ROS 2 工作区中有 [ROS 2 设置](../ros2/user_guide.md) 与 [`px4_msgs`](https://github.com/PX4/px4_msgs]。

2. 将代码仓库克隆到工作空间中

   ```sh
   cd $ros_workspace/src
   git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
   ```

   提示信息
   为确保兼容性，请使用 PX4、px4_msgs（PX4 消息包）及该库的最新 main 分支。
   另请参阅 [here]（https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4）

:::

3. 构建工作空间:

   ```sh
   cd ..
   colcon building
   source install/setup.bash
   ```

4. 在另一个外壳中，启动 PX4 SITL：

   ```sh
   cd $px4-autopilot
   make px4_sitl gazebo-classic
   ```

   (这里我们使用Gazebo-Classic，但你可以使用任何模型或模拟器)

5. 在新的 shell 中运行微XRCE 代理 (您可以在以后继续运行)：

   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```

6. 启动QGroundControl。

   ::: info
   Use QGroundControl Daily, which supports dynamically updating the list of modes.

:::

7. 回到ROS2终端，运行一个示例模式：

   ```sh
   ros2 运行 example_mode_manual_cpp example_mode_manual
   ```

   你应会看到类似如下的输出，其中显示 “我的手动模式"已注册：

   ```sh
   [DEBUG] [example_mode_manual]: Checking message compatibility...
   [DEBUG] [example_mode_manual]: Subscriber found, continuing
   [DEBUG] [example_mode_manual]: Publisher found, continuing
   [DEBUG] [example_mode_manual]: Registering 'My Manual Mode' (arming check: 1, mode: 1, mode executor: 0)
   [DEBUG] [example_mode_manual]: Subscriber found, continuing
   [DEBUG] [example_mode_manual]: Publisher found, continuing
   [DEBUG] [example_mode_manual]: Got RegisterExtComponentReply
   [DEBUG] [example_mode_manual]: Arming check request (id=1, only printed once)
   ```

8. 在 PX4 外壳上，您可以检查 PX4 是否注册了新模式：

   ```sh
   指挥官状态
   ```

   输出应包含：

   ```plain
   INFO  [commander] Disarmed
   INFO  [commander] navigation mode: Position
   INFO  [commander] user intended navigation mode: Position
   INFO  [commander] in failsafe: no
   INFO  [commander] External Mode 1: nav_state: 23, name: My Manual Mode
   ```

9. 在这一点上，您也应该能够在 QGroundControl 中看到模式：

   ![QGC Modes](../../assets/middleware/ros2/px4_ros2_interface_lib/qgc_modes.png)

10. 选择该模式，确保你拥有手动控制源（物理或虚拟操纵杆），并为载具解锁
    然后模式将激活，它将打印以下输出：

    ```sh
    [DEBUG] [example_mode_manual]: 模式“我的手动模式” 已激活
    ```

11. 现在您已准备好创建自己的模式。

## 如何使用代码库

以下各节介绍该库提供的特定功能。
此外，任何其他PX4主题都可以订阅或发布。

### 模式类定义

此部分通过如何为自定义模式创建类的示例。

若要完整的应用程序，请查看[示例在 `Auterion/px4-ros2-interfacee-lib` 仓库中](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp)，例如[示例/cpp/mod/manual](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/examples/cpp/modes/manual/include/mode.hpp)。

```cpp{1,5,7-9,24-31}
class MyMode : public px4_ros2::ModeBase // [1]
{
public:
  explicit MyMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"My Mode"}) // [2]
  {
    // [3]
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
  }

  void onActivate() override
  {
    // Called whenever our mode gets selected
  }

  void onDeactivate() override
  {
    // Called when our mode gets deactivated
  }

  void updateSetpoint(const rclcpp::Duration & dt) override
  {
    // [4]
    const Eigen::Vector3f thrust_sp{0.F, 0.F, -_manual_control_input->throttle()};
    const Eigen::Vector3f rates_sp{
      _manual_control_input->roll() * 150.F * M_PI / 180.F,
      -_manual_control_input->pitch() * 150.F * M_PI / 180.F,
      _manual_control_input->yaw() * 100.F * M_PI / 180.F
    };
    _rates_setpoint->update(rates_sp, thrust_sp);
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
};
```

- `[1]`: 首先创建一个从 [`px4_ros2::ModeBase`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1ModeBase.html)继承的类。
- `[2]`: 在构造函数中，我们传递模式名称。 这也使我们能够配置一些其他内容，例如替换飞行控制器的内置模式。
- `[3]`：我们在此处创建后续想要使用的所有对象。
  这可以是 RC 输入、设置点类型(s)或遥测数据。 `*this` 作为`Context`传递给每个对象，将对象与模式联系起来。
- `[4]`：每当该模式处于激活状态时，此方法会定期被调用（更新频率取决于设定值类型）。
  我们可以在这里开展工作并产生一个新的设定点。

创建此模式的实例后， `mode->doRegister()` 必须被调用，在飞行控制器中进行实际注册，如果失败则返回 `false` 。
如果使用模式执行器，`doRegister()`必须调用模式执行器而不是模式。

### 模式执行器类定义

本节逐步讲解如何创建模式执行器类的示例。

```cpp{1,4-5,9-16,20,33-57}
class MyModeExecutor : public px4_ros2::ModeExecutorBase // [1]
{
public:
  MyModeExecutor(px4_ros2::ModeBase & owned_mode) // [2]
  : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(owned_mode.node())
  { }

  enum class State // [3]
  {
    Reset,
    TakingOff,
    MyMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::TakingOff, px4_ros2::Result::Success); // [4]
  }

  void onDeactivate(DeactivateReason reason) override { }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      return;
    }

    switch (state) { // [5]
      case State::Reset:
        break;

      case State::TakingOff:
        takeoff([this](px4_ros2::Result result) {runState(State::MyMode, result);});
        break;

      case State::MyMode: // [6]
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed([this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};
```

- `[1]`: 首先创建一个继承 [`px4_ros2::ModeExecutorBase`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1ModeExecutorBase.html)。
- `[2]`: 构造函数采用与执行器相关联的自定义模式，并传递给`ModeExecutorBase`的构造函数。
- `[3]`: 我们为想要运行的状态定义一个枚举。
- `[4]`: `onActivate` 在执行器激活时被调用。 此时，我们可以开始遍历这些状态了。
  你是如何操作的，在这个示例中使用 `runState` 方法来执行下一个状态。
- `[5]`: 在切换到状态时，我们会调用 `ModeExecutorBase` 中的异步方法来启动所需的模式：`run`, `eff`、`rtl`、“等等”。
  这些方法被传递了一个被要求完成的函数； 回调提供一个 `Result` 参数，告诉您操作是否成功。
  回调运行下一个成功状态。
- `[6]`: 我们使用 `scheduleMode()` 方法来启动执行者"拥有模式", 遵循与其他状态处理器相同的模式。

### 设置点类型

模式可选择其用于控制载具的设定值类型。
所用类型还界定了与不同类型载具的兼容性。

以下章节提供了支持的设置点类型列表：

- [MulticopterGotoSetpointType](#go-to-setpoint-multicoptergotosetpointtype): <Badge type="warning" text="MC only" /> 平滑的位置控制以及（可选的）航向控制
- [FwLateralLongitudinalSetpointType](#fixed-wing-lateral-and-longitudinal-setpoint-fwlaterallongitudinalsetpointtype): <Badge type="warning" text="FW only" /> <Badge type="tip" text="main (planned for: PX4 v1.17)" /> 对横向和纵向固定翼动态的直接控制
- [DirectActuatorsSetpointType](#direct-actuator-control-setpoint-directactuatorssetpointtype)：直接控制发动机和飞行地面servo setpoints
- [Rover Setpoints](#rover-setpoints): <Badge type="tip" text="main (planned for: PX4 v1.17)" />直接访问火星车控制设定值（位置、速度、姿态、速率、油门和转向）。

:::tip
其他设置点类型目前是实验性的，可在以下网址找到：[px4_ros2/control/setpoint_types/experimental](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/px4_ros2_cpp/include/px4_ros2/control/setpoint_types/experimental)。

您可以通过添加一个从 `px4_ros2::SetpointBase` 继承的类来添加您自己的 setpoint 类型， 根据设置点的要求设置配置标志，然后发布任何包含设置点的主题。
:::

#### 转到设置点 (MulticopterGotoSetpointType)

<Badge type="warning" text="MC only" />

<Badge type="warning" text="Multicopter only" />

:::info
当前，此设定点类型仅支持多旋翼飞行器。
:::

可通过[`px4_ros2::MulticopterGotoSetpointType`](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/control/setpoint_types/multicopter/goto.hpp) 设定点类型，对位置设定点以及（可选的）航向设定点进行平滑控制。
设定点类型会被传输至飞控主模块（FMU），该模块基于采用时间最优、最大加加速度轨迹构建的位置及航向平滑器。

还有一个 [`px4_ros2::MulticopterGotoGlobalSetpointType`](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/control/setpoint_types/multicopter/goto.hpp)， 该类支持在全局坐标系下发送设定点。

最简单的用法就是直接向update method中输入一个3D 位置

```cpp
const Eigen::Vector3f target_position_m{-10.F, 0.F, 3.F};
_goto_setpoint->update(target_position_m);
```

在这种情况下，航向将保持不受控制的状态。
若要额外控制航向，可将其指定为第二个输入参数：

```cpp
const Eigen:::Vector3f target_position_m{-10.F, 0.F, 3.F};
const float heading_rad = 3.14F;
_goto_setpoint->update(
  target_position_m,
  heading_rad);
```

“前往设定值”（go-to setpoint）的一项额外功能是，可对底层平滑控制器的速度限制进行动态控制（即对最大水平和平动速度、最大垂直平动速度以及航向速率进行动态控制）。
若如上文所述未指定（该参数），则平滑控制器将默认采用设备的默认最大值（通常设定为其物理极限值）
平滑控制器仅会降低速度限制，绝不会提高速度限制。

```cpp
_goto_setpoint->update(
  target_position_m,
  heading_rad,
  max_水平_velocity_m_s,
  max_vertical_velocity_m_s,
  max_heading_rate_rad_s);
```

更新方法中，除位置外的所有参数均被模板化为 `std::optional<float>` 类型。这意味着，若需限制航向速率但不限制平动速度，可通过 `std::nullopt` 实现这一需求。

```cpp
_goto_setpoint->update(
  target_position_m,
  heading_rad,
  std::nullopt,
  std::nullopt,
  max_heading_rate_rad_s);
```

#### 固定翼横向与纵向设定值（FwLateralLongitudinalSetpointType，固定翼横向纵向设定值类型）

<Badge type="warning" text="Fixed wing only" /> <Badge type="tip" text="main (planned for: PX4 v1.17)" />

:::info
此设定值类型支持固定翼飞行器，以及处于固定翼模式下的垂直起降飞行器（VTOL）。
:::

使用[`px4_ros2::FwLateralLongitudinalSetpointType`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1FwLateralLongitudinalSetpointType.html)直接控制固定翼飞行器的横向与纵向动力学特性，即分别控制其侧向运动（转弯 / 倾斜）和前向 / 垂直运动（加速及爬升 / 下降）。
这个设置点被传输到 PX4 [_FwLateralLongitudinalControl_ module](../modules/modules_controller.md#fw-lat-lon-control)，该模块会对横向与纵向输入进行解耦处理，同时确保不超出飞行器的各项限制范围。

为了控制载具，必须提供至少一个横向**和**一个纵向设定值：

1. 在纵向输入中：必须至少有一个 “高度”（altitude）或 “高度速率”（height_rate）为有限值，才能实现垂直运动控制。
   若二者均设为 NAN，则载具将保持当前高度。
2. 在横向输入中：“航线角”（course）、“空速方向”（airspeed_direction）或 “横向加速度”（lateral_acceleration）这三者中，至少有一个必须为有限值。

关于可控参数的详细说明，请参考消息定义([FixedWingLateralSetpoint](../msg_docs/FixedWingLateralSetpoint.md) 和 [FixedWingLongitudinalSetpoint](../msg_docs/FixedWingLongitudinalSetpoint.md))。

##### 基本用法

该类型包含多个更新方法，每种方法可支持你指定的设定值数量逐步增加。

最简单的方法是 updateWithAltitude()，该方法可让你指定 “航线角”（course）和 “高度”（altitude）这两个目标设定值。

```cpp
const float altextede_msl = 500.F;
const float course = 0.F; // due North
_fw_later_longitudinal_setpoint->updateWAltitude(altyde_msl, course);
```

PX4 会利用这些设定值计算出横滚角（roll angle）、俯仰角（pitch angle）和油门（throttle）设定值，并将其发送至底层控制器。
需注意，使用此方法时，预期的指令飞行会相对平缓 / 不激进。
此操作按如下方式执行：

- 横向控制输出：

  航线角设定值（由用户设定）&rarr; 空速方向 (航向) 设定值 &rarr; 横向加速设定值 &rarr; 滚动角度设定值

- 纵向控制输出：

  高度设定值（由用户设定） &rarr;  高度速率设定值 &rarr; 俯仰角设定值及油门设定。

updateWithHeightRate() 方法允许你设置目标 “航线角”（course）和 “高度速率”（height_rate）（当爬升或下降速度至关重要，或需要对其进行动态控制时，此方法非常实用）：

```cpp
const float height_rate = 2.F;
const float course = 0.F; //due North
_fw_lateral_longitudinal_setpoint->updateWheightRate(high_rate course);
```

updateWithAltitude() 和 updateWithHeightRate() 这两种方法还允许你分别将 “等效空速”（equivalent airspeed）或 “横向加速度”（lateral acceleration）指定为第三个和第四个参数，从而对其进行控制：

```cpp
const float altitude_msl = 500.F;
const float course = 0.F; // due North
const float equivalent_aspd = 15.F; // m/s
const float lateral_acceleration = 2.F; // FRD, used as feedforward

_fw_lateral_longitudinal_setpoint->updateWithAltitude(altitude_msl,
  course,
  equivalent_aspd,
  lateral_acceleration);
```

等效的空速和横向加速参数定义为 `std::optional<float>`, 所以你可以通过 `std::nullopt` 省略其中任何一个。

:::tip
若同时提供了横向加速度设定值和航线角设定值，横向加速度设定值将被用作前馈（feedforward）
:::

##### 使用设定值结构体实现完全控制

为实现充分的灵活性，你可以创建并传递一个[`FwLateralLongitudinalSetpoint` ](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1FwLateralLongitudinalSetpoint.html) 结构体。
每个字段都使用 `std::optional<float>` 模板。

:::tip
若同时设置了航线角（course）和空速方向（airspeed direction），则空速方向优先，航线角不进行控制。
若航线角或空速方向中任意一个为有限值，则横向加速度会被视为前馈。
若同时设置了高度和高度速率，则高度速率优先，高度不进行控制。
:::

```cpp
px4_ros2:::FwLateralLongitudinalSetpoint setpoint_s;

settpoint_s.withCourse(0.F);
// setpoint_s.withAirspeedDirection(0.2F); // 失控
setpoint_s.withLateralActivation(2.F); // feedforward
/setpoint_s.withAltude(500.F); // 失控
setpoint_s.withightRate(2.F);
settpoint_s.withequivalentAirspeed(15.F);

_fw_lateral_longitudinal_sett->upate(settpoint_
```

下图展示了在所有输入均已设定的情况下，FwLateralLongitudinalSetpointType 与 PX4 之间的交互关系。

![FW ROS交互](../../assets/middleware/ros2/px4_ros2_interface_lib/fw_lat_long_ros_interaction.svg)

##### 高级配置（可选）

你还可以传递一个[`FwControlConfiguration`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1FwControlConfiguration.html)结构体以及设定值，以覆盖默认的控制器设置和约束条件，例如俯仰角限制、油门限制以及目标下降 / 爬升速率。
这是针对高级用户的：

```cpp
px4_ros2::FwLateralLongitudinalSetpoint setpoint_s;

setpoint_s.withAirspeedDirection(0.F);
setpoint_s.withLateralAcceleration(2.F); // feedforward
setpoint_s.withAltitude(500.F);
setpoint_s.withEquivalentAirspeed(15.F);

px4_ros2::FwControlConfiguration config_s;

config_s.withTargetClimbRate(3.F);
config_s.withMaxAcceleration(5.F);
config_s.withThrottleLimits(0.4F, 0.6F);

_fw_lateral_longitudinal_setpoint->update(setpoint_s, config_s);
```

所有配置字段都定义为 `std::optional<float>`。
未设置的值将默认采用 PX4 的配置。
更多关于配置选项的信息，请参阅 [LateralControlConfiguration](../msg_docs/LateralControlConfiguration.md)和 [FixedWingLongitudinalConfiguration](../msg_docs/LongitudinalControlConfiguration.md)。

:::info
为保障安全，PX4 会自动将配置值限制在飞行器的约束范围内。
例如，油门覆盖值会被限制在 [`FW_THR_MIN`](../advanced_config/parameter_reference.md#FW_THR_MIN)
和[`FW_THR_MAX`](../advanced_config/parameter_reference.md#FW_THR_MAX)之间。
:::

#### 直接执行器控制设定点（DirectActuatorsSetpointType）

可以使用 [px4_ros2::DirectActuatorsSetpointType](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1DirectActuatorsSetpointType.html) 设置点类型直接控制执行器。
电机和舵机可独立设置。
请注意，该分配（设置 / 指派）具有载具和配置特定性。
例如，要控制一架四旋翼飞行器，你需要根据其 [输出配置] (../concept/control_allocation.md)来设置前 4 个电机。

:::info
若你想控制的执行器并非用于控制飞行器的运动（例如，而是用于控制有效载荷舵机），请参阅 [below](#controlling-an-independent-actuator-servo)。
:::

#### Rover 设置点

<Badge type="tip" text="main (planned for: PX4 v1.17)" /> <Badge type="warning" text="Experimental" />

滚动模块使用层次结构来传播设置点：

![Rover Control Structure](../../assets/middleware/ros2/px4_ros2_interface_lib/rover_control_structure.svg)

:::info
所提供的“highest”设定值将被用于 PX4 机器人模块中，以生成低于该值的设定值（并对其进行覆盖！）。
这个层次结构有提供有效控制输入的明确规则：

- 提供一个位置集点，**or**
- “左”上的设置点之一(速度 **或** 节点) **和** “右”上的设置点之一(态度、速率 **或** 节点)。 所有“左”和“右”设置点的组合都是有效的。

为了便于使用，我们以新的 SettpointType 的形式揭示这些有效的组合。
:::

通过控制界面暴露的 RoverSetpointTypes 是这些设置点的组合，导致有效的控制输入：

| SetpointType                                                                                                                        | 安装位置                        | Speed                                            | 油门                                               | Attitude                                         | 频率                                               | Steering                                         | Control Flags                                          |
| ----------------------------------------------------------------------------------------------------------------------------------- | --------------------------- | ------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------------ |
| [RoverPosition](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverPositionSetpointType.html#details)         | &check; | (&check;) | (&check;) | (&check;) | (&check;) | (&check;) | Position, Velocity, Attitude, Rate, Control Allocation |
| [RoverSpeedAttitude](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverSpeedAttitudeSetpointType.html)       |                             | &check;                      | (&check;) | &check;                      | (&check;) | (&check;) | Velocity, Attitude, Rate, Control Allocation           |
| [RoverSpeedRate](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverSpeedRateSetpointType.html)               |                             | &check;                      | (&check;) |                                                  | &check;                      | (&check;) | Velocity, Rate, Control Allocation                     |
| [RoverSpeedSteering](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverSpeedSteeringSetpointType.html)       |                             | &check;                      | (&check;) |                                                  |                                                  | &check;                      | Velocity, Control Allocation                           |
| [RoverThrottleAttitude](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverThrottleAttitudeSetpointType.html) |                             |                                                  | &check;                      | &check;                      | (&check;) | (&check;) | Attitude, Rate, Control Allocation                     |
| [RoverThrottleRate](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverThrottleRateSetpointType.html)         |                             |                                                  | &check;                      |                                                  | &check;                      | (&check;) | Rate, Control Allocation                               |
| [RoverThrottleSteering](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1RoverThrottleSteeringSetpointType.html) |                             |                                                  | &check;                      |                                                  |                                                  | &check;                      | Control Allocation                                     |

&check; 是我们发布的设置点，(&check;) 是根据上面的层次结构由 PX4 旋转模块内部生成的。

使用 `RoverSpeedAttitude SettpointType` 的特定驱动器模式示例为 [here](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/modes/rover_velocity)。

### 控制VTOL

<Badge type="tip" text="main (planned for: PX4 v1.17)" /> <Badge type="warning" text="Experimental" />

要在外部飞行模式下控制VTOL，需确保根据当前飞行配置返回正确的设定值类型：

- 多旋翼模式：使用与多旋翼控制兼容的设定值类型。 例如：要么[`MulticopterGotoSetpointType`](#go-to-setpoint-multicoptergotosetpointtype)要么[`TrajectorySetpointType`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1TrajectorySetpointType.html)。
- 固定翼形模式：使用 [`FwLateralLongitudinalSetpointType`](#fixed-wing-lateral-and-longitudinal-setpoint-fwlaterallongitudinalsetpointtype)。

只要VTOL在整个外部模式期间始终处于多旋翼模式或固定翼模式中的任意一种，就无需额外处理。

如果您想要在您的外部模式中命令一个 VTAL 过渡，您需要使用 [VTOL API](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1VTOL.html)。 VTAL API具备下达转换指令和查询载具当前状态的功能。

谨慎使用此 API ！
通过外部下达转换指令时，用户需部分负责确保（飞行器）运行平稳且安全；这与机上转换（例如通过遥控器开关实现）不同，在机上转换模式下，整个过程由 PX4 全权处理。

1. 确保你的模式可调用 TrajectorySetpointType（轨迹设定值类型）和 FwLateralLongitudinalSetpointType（固定翼横纵向设定值类型）这两种设定值类型。
2. 在您模式的构造函数中创建 `px4_ros2::VTOL` 的实例。
3. 要下达转换指令，你可以在 VTOL 对象上调用 toMulticopter() 或 toFixedwing() 方法，以设置所需状态。
4. 在转换过程中，发送以下设定值组合：

   ```cpp
   // Assuming the instance of the px4_ros2::VTOL object is called vtol

   // Send TrajectorySetpointType as follows:
   Eigen::Vector3f acceleration_sp = vtol.computeAccelerationSetpointDuringTransition();
   Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};

   _trajectory_setpoint->update(velocity_sp, acceleration_sp);

   // Send FwLateralLongitudinalSetpointType with lateral input to realign vehicle as desired

   float course_sp = 0.F; // North

   _fw_lateral_longitudinal_setpoint->updateWithAltitude(NAN, course_sp)
   ```

   这将确保转换过程在 PX4 系统内部得到妥善处理。
   你可以选择性地将一个减速度设定值传递给 computeAccelerationSetpointDuringTransition()，以便在反向转换过程中使用。

要查询载具的当前状态，可在 px4_ros2::VTOL 对象上调用 getCurrentState() 方法。

请参阅[此外部飞行模式实现](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/modes/vtol) 有关如何使用此 API 的实际示例。

### 控制独立执行器/Servo

如果您想要控制一个独立执行器(aservo)，遵循以下步骤：

1. [Configure the output](../payloads/generic_actuator_control.md#generic-actuator-control-with-mavlink).
2. 在您的模式的构造函数中创建一个实例[px4_ros2::PeripheralActorControls](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1PeripheralActuatorControls.html)。
3. 调用`set()` 方法以控制执行器
   此操作可独立于任何活跃的设定点执行。

### 数传

你可以通过以下类直接访问 PX4 遥测主题：

- [OdometryGlobalPosition](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1OdometryGlobalPosition.html): 全球位置
- [OdometryLocalPosition](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1OdometryLocalPosition.html): 本地位置、速度、加速度和航向
- [OdometryAttitude](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1OdometryAttitude.html): 载具状态
- [OdometryAirspeed](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1OdometryAirspeed.html):空速

例如，你可以通过以下方式查询飞行器当前的位置估算值：

```cpp
std::shared_ptr<0> _vehicle_local_position;
...

// Get vehicle's last local position
_vehicle_local_position->positionNed();

// Check last horizontal position is valid
_vehicle_local_position->positionXYValid();
```

:::info
这些主题围绕内部的 PX4 主题提供了一个包装程序，使代码库能够在内部主题发生变化时保持兼容性。
检查 [px4_ros2/odometry](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/px4_ros2_cpp/include/px4_ros2/odometry) 新主题，当然你也可以使用任何由 PX4 发布的 ROS 2 主题。
:::

### 故障保护与模式要求

每种模式都有一组需求标志。
这些（需求标志）通常会根据模式语境下所使用的对象自动设置。
例如，当通过以下代码添加手动控制输入时，手动控制的需求标志会被设置：

```cpp
_manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
```

具体而言，在 PX4 中，若条件未满足，设置某个标志会产生以下结果：

- 在选定模式时不允许进行解锁操作
- 当已处于武装状态时，该模式无法被选择。
- 当载具已解锁且该模式被选中时，相关的故障保护机制会被触发（例如，针对手动控制需求的遥控器信号丢失故障保护）。
  检查  [safety page] (../config/safety.md) 如何配置故障安全行为。
  当某模式已被选中，且该模式发生崩溃或失去响应时，也会触发故障保护机制。

这是手动控制标志对应的流程图：

![Mode requirements diagram](../../assets/middleware/ros2/px4_ros2_interface_lib/mode_requirements_diagram.png)

<!-- source: https://drive.google.com/file/d/1g_NlQlw7ROLP_mAi9YY2nDwP0zTNsFlB/view -->

在模式注册后，可以手动更新任意模式要求。
例如，要将返航点（home position）添加为一项要求：

```cpp
modeRequirements().home_position = true;
```

标记的完整列表可以在 [requirement_flags.hpp](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/common/requirement_flags.hpp 中找到。

#### 延迟故障保护

模式或模式执行器可以通过调用该方法暂时延迟非必要的故障保护。 [`deferFailsafesSync()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1ModeExecutorBase.html#a16ec5be6ebe70e1d0625bf696c3e29ae).
若想在故障保护即将被触发时收到通知，需重写该方法。 [`void onFailsafeDeferred()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1ModeExecutorBase.html#ad80a234c8cb2f4c186fa2b7bffd1a1dd).

例如检查[integration test](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/test/integration/overrides.cpp)。

### 将模式指派给 RC 切换或操纵动作

外部模式可以被分配给[RC switches](../config/flight_mode.md) 或操纵杆动作。
当将模式分配给RC开关时，你需要知道其索引（因为参数元数据中不包含动态模式名称）。
在模式运行期间，使用 'commander status'（指令状态）来获取该信息。

例如：

```plain
   INFO  [commander] External Mode 1: nav_state: 23, name: My Manual Mode
```

意味着你需要在 QGC中选择外部模式 1（External Mode 1） ：

![QGC Modes](../../assets/middleware/ros2/px4_ros2_interface_lib/qgc_mode_assignment.png)

:::info
PX4通过存储模式名称的散列确保指定模式始终分配到同一索引。
这使它独立于多个外部模式下的启动订购。
:::

### 替换内部模式

外部模式可替换现有的内部模式，例如[Return](../flight_modes/return.md) mode (RTL).
这样做，每当选择RTL(通过用户或故障安全情况)，就使用外部模式而不是内部模式。
当外部模式失去响应或发生崩溃时，内部模式仅用作备用模式。

替换模式可在' ModeBase '构造函数的设置中进行配置：

```cpp
设置{kName, false, ModeBase::kModeIDRtl}
```
