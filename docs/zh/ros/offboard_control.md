# Offboard控制

:::warning
[Offboard control](../flight_modes/offboard.md) is dangerous.
这是通过 Mavlink 协议完成的, 特别是 <a href="https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED">SET_POSITION_TARGET_LOCAL_NED</a> 和 <a href="https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET">SET_ATTITUDE_TARGET</a> 消息。
:::

Offboard控制背后的想法是能够使用在自动驾驶仪外运行的软件来控制 PX4 飞控。
This is done through the MAVLink protocol, specifically the [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) and the [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) messages.

## Offboard控制固件设置

在进行Offboard开发前，您需要在固件端做两个设置。

### Enable RC Override

In _QGroundControl_ you can set the [COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE) parameter to automatically switch from offboard mode (or any mode) to Position mode if the RC sticks are moved.
This is the best way to ensure that an operator can easily take control of the vehicle and switch to the safest flight mode.

### 1. 将遥控开关映射到离板模式激活

In _QGroundControl_ you can set the [RC_MAP_OFFB_SW](../advanced_config/parameter_reference.md#RC_MAP_OFFB_SW) parameter to the RC channel that will be used to activate offboard mode.
This can be used to switch between offboard mode and the mode set by the mode switch ([RC_MAP_MODE_SW](../advanced_config/parameter_reference.md#RC_MAP_MODE_SW)).
You can also switch into offboard mode using a GCS/MAVLink so this is not "mandatory".

Note also that this mechanism is not as "safe" as using [RC Override](#enable-rc-override) to switch out of offboard mode, because the mode you switch to is unpredictable.

### 2. 启用机载计算机接口

Enable MAVLink on the serial port that you connect to the companion computer (see [Companion Computers](../companion_computer/index.md)).

## 硬件设置

通常，有三种方式设置Offboard模式通信。

### 串口电台

1. 一端连接飞控的 UART
2. 一端连接地面站电脑

  参考电台包括：

  - [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
  - [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

[![Mermaid graph: mavlink channel](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)

<!-- original mermaid graph
graph TD;
  gnd[Ground Station] --MAVLink-- > rad1[Ground Radio];
  rad1 --RadioProtocol-- > rad2[Vehicle Radio];
  rad2 --MAVLink-- > a[Autopilot];
-->

### 板载处理器

在飞行器上部署一台小型将计算机，用 UART 转 USB 适配器连接飞控。
这里有许多可能性，这将取决于您除了向自驾仪发送指令外，还想要做什么样的额外机载处理。
Some examples are provided in [Companion Computers](../companion_computer/index.md#companion-computer-options).

[![Mermaid diagram: Companion mavlink](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

<!-- original mermaid graph
graph TD;
  comp[Companion Computer] --MAVLink-- > uart[UART Adapter];
  uart --MAVLink-- > Autopilot;
-->

### On-board processor and wifi link to ROS (**_Recommended_**)

部署在飞行器上的小型计算机除了通过 UART - USB 适配器连接到自动驾驶仪外，同时还可通过WiFi连接至运行 ROS 的地面站。
这可以是上述配备了WiFi适配器的任一计算机。

[![Mermaid graph: ROS](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgc3ViZ3JhcGggR3JvdW5kICBTdGF0aW9uXG4gIGduZFtST1MgRW5hYmxlZCBDb21wdXRlcl0gLS0tIHFnY1txR3JvdW5kQ29udHJvbF1cbiAgZW5kXG4gIGduZCAtLU1BVkxpbmsvVURQLS0-IHdbV2lGaV07XG4gIHFnYyAtLU1BVkxpbmstLT4gdztcbiAgc3ViZ3JhcGggVmVoaWNsZVxuICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuICB1YXJ0IC0tLSBBdXRvcGlsb3RcbiAgZW5kXG4gIHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVERcbiAgc3ViZ3JhcGggR3JvdW5kICBTdGF0aW9uXG4gIGduZFtST1MgRW5hYmxlZCBDb21wdXRlcl0gLS0tIHFnY1txR3JvdW5kQ29udHJvbF1cbiAgZW5kXG4gIGduZCAtLU1BVkxpbmsvVURQLS0-IHdbV2lGaV07XG4gIHFnYyAtLU1BVkxpbmstLT4gdztcbiAgc3ViZ3JhcGggVmVoaWNsZVxuICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuICB1YXJ0IC0tLSBBdXRvcGlsb3RcbiAgZW5kXG4gIHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)

<!-- original mermaid graph
graph TD
  subgraph Ground  Station
  gnd[ROS Enabled Computer] --- qgc[qGroundControl]
  end
  gnd --MAVLink/UDP-- > w[WiFi];
  qgc --MAVLink-- > w;
  subgraph Vehicle
  comp[Companion Computer] --MAVLink-- > uart[UART Adapter]
  uart --- Autopilot
  end
  w --- comp
-->
