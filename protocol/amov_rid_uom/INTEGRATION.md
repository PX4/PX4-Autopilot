# M4 三端联调说明

本阶段覆盖统一接口、RID → PX4 → QGC 身份读取、离线激活、模拟 UOM 报送，以及 PX4 解锁综合检查。正式 UOM 服务适配、公司签发后台和最终实机验收不在本阶段自动测试结论内。

## 配套版本与产物

| 仓库 | 联调配置 | 生产配置 |
| --- | --- | --- |
| PX4-Autopilot | `amovlab_flycore_ridbench`：允许模拟 UOM 满足检查 | `amovlab_flycore_rid`：拒绝模拟 UOM |
| ArduRemoteID | `AMOV_RID_TEST`：只信任公开测试激活根 | `AMOV_RID`：需要公司激活公钥，默认无根时拒绝激活 |
| QGroundControl | `QGC_UOM_BENCH_TEST=ON`，仅 localhost 模拟服务 | 默认 `OFF`，正式 provider 未配置时不可就绪 |

三个仓库的 `protocol/amov_rid_uom` 必须来自同一交付版本。不能混用旧 QGC、旧 RID 参数设置流程或未扩展的 MAVLink 消息表。

本机联调入口：

- 飞控：`/home/amovlgf/PX4-Autopilot/build/amovlab_flycore_ridbench/amovlab_flycore_ridbench.px4`。
- RID：`/home/amovlgf/ArduRemoteID/RemoteIDModule/ArduRemoteID-AMOV_RID_TEST.bin`。
- QGC：`/home/amovlgf/QGroundControl/tools/amov_m4/run_qgc.sh`。
- RID 激活说明：`/home/amovlgf/ArduRemoteID/docs/RID_ACTIVATION.md`。

RID 全量 `.bin` 从 0x0 烧写会覆盖 NVS，不能当作保留授权升级。`_OTA.bin` 是应用分区镜像，仍受既有 OTA 签名规则约束；激活测试密钥不能签发 OTA 固件。此次开发未烧写设备或修改 eFuse。

## 先跑通正常流程

1. 使用上述三份联调配置，在地面无桨台架验证。RID 与飞控 CAN 波特率一致，节点 ID 不重复。示例飞控节点为 1。
2. 确认飞控参数 `UAVCAN_ENABLE=2`、`UAVCAN_NODE_ID=1`、`COM_ARM_ODID=2`、`COM_RID_UOM=1`。配置提供默认值，设备原有持久化参数仍须检查；更改 CAN 参数后重启。`COM_RID_UOM=0` 不属于本验收配置。
3. 连接 RID Wi-Fi，在网页读取真实 12 位 UID。首次未激活时，身份查询和管理功能应工作，业务广播及正常解锁均不就绪。
4. 签发测试授权。以下 UID 是样例，必须替换为设备网页中的实际值；产品码恰为 20 个非空格 ASCII 字符。

    ```sh
    cd /home/amovlgf/ArduRemoteID
    python3 scripts/activation_package.py sign-test \
      --uid AABBCCDDEEFF --product AMOV0123456789012345 \
      --registration REG-12345678 --uin 12345678 --class 2 \
      --revision 1 --output /tmp/amov-device-rev1.act
    ```

5. 在网页上传 `.act`，等待查询结果为 `PENDING_REBOOT` 且 `busy=false`。HTTP 202 仅表示接收，不能作为激活成功判据。未冷上电前，有效身份保持原值。
6. 人工完全断电再上电，核对 `state=1`、有效 revision、产品码及 hash。软件复位、看门狗、brownout 和 OTA 重启不应用 pending。硬件无法区分人工完全断电与意外完全断电。
7. 配置 RID 的两个非身份参数：`CN_CAN_NODE` 必须等于飞控的 CAN 节点，`CN_WIFI_ID` 必须是目标无线封装标识。全新/全量烧写设备默认分别是 0 和空字符串，不配置即使激活成功也不会就绪。这两个参数仅从 CAN 维护入口设置，不能通过普通 QGC 数值参数桥配置字符串。使用已准备好的 SocketCAN `can0` 和实际 RID 节点；以下以节点 125 为例：

    ```sh
    cd /home/amovlgf/ArduRemoteID
    python3 scripts/cn_dronecan_test.py can0 --node-id 42 --target-node 125 --no-stream \
      --set CN_CAN_NODE=1 --set CN_WIFI_ID=02000001
    ```

    工具节点 42 须与其他节点不同，`02000001` 仅为封闭台架的实验封装示例，正式互通标识须实测确认。如果 `LOCK_LEVEL>0`，先按现有维护授权流程使用设备信任的密钥操作；不绕过参数锁。不要再设置受签名保护的 `CN_UAS_ID/CN_UIN/CN_CLASS/RID_PROFILE`。详细维护流程见 RID 仓库 `docs/CN_CAN_WIFI_OTA.md`，其旧身份配置步骤已被本次激活流程替代。

8. 启动本地模拟服务，然后启动本次 QGC：

    ```sh
    cd /home/amovlgf/QGroundControl
    python3 tools/uom_mock_server.py --port 8765
    ```

    另开一个终端：

    ```sh
    cd /home/amovlgf/QGroundControl
    bash tools/amov_m4/run_qgc.sh
    ```

9. 通过 UDP 或串口连接飞控。容器使用主机网络；串口需将真实设备显式传入，例如 `QGC_SERIAL_DEVICE=/dev/ttyACM0 bash tools/amov_m4/run_qgc.sh`。插拔后重新启动脚本。QGC 独立配置保存在 `~/.local/share/amov-m4-qgc`。
10. 准备有效飞行器定位、UTC，以及有效操作员位置。若使用固定操作员位置，明确设置实际经纬度和椭球高；不要以零值或海拔代替未知椭球高。冻结或乱序遥测不能续期。
11. 在 QGC Remote ID 状态面板核对产品码、完整登记号、RID UID、revision、pending 与模拟标识。应先完成身份绑定和鉴权，再收到模拟服务 ACK，最终进入报送状态。
12. 核对 PX4 RID/UOM 检查通过。其他飞行器检查仍独立生效，不能把“RID/UOM 就绪”当作“飞机全部可飞”。普通解锁依赖身份、激活、同一 RID 节点新鲜 ArmStatus 和新鲜 UOM ACK；既有快速再解锁/旁路策略保持原行为。

真实 UID、正式登记号或生产私钥不要写入测试仓库。测试工具只含公开测试种子。

## 故障注入

保持模拟服务运行，在另一个终端注入上报失败：

```sh
curl --fail --silent --show-error http://127.0.0.1:8765/control/fault \
  -H 'Content-Type: application/json' \
  --data '{"fault":"unavailable","phase":"report","count":100}'
```

恢复正常：

```sh
curl --fail --silent --show-error http://127.0.0.1:8765/control/fault \
  -H 'Content-Type: application/json' --data '{"fault":"none","count":0}'
```

也可用 `unauthorized`、`reject`、`malformed`、`mismatch`，或 `delay` 配合 `delay_ms=4000`。模拟服务仅监听 localhost，返回结果不是正式 UOM 受理凭证。

## 用户实机验收表

| 编号 | 操作 | 通过判据 |
| --- | --- | --- |
| H01 | 未激活、没有 RID、断 CAN、同时接入两个 RID | 正常解锁检查失败，显示具体身份或健康原因 |
| H02 | 上传正确包，不断电；再软件复位；最后冷上电 | 前两步有效身份不变，冷上电后才生效 |
| H03 | 错设备、错签名、截断、超长、旧版本、同版本异内容 | 拒绝且保留原有效身份；完全相同原包可幂等重试 |
| H04 | 上传更高 revision，观察 QGC 再冷上电 | pending 单独显示；生效后旧身份/会话/ACK 全部失效并重新鉴权 |
| H05 | 持续上传/查询网页，同时观测 CAN 和无线 | 不阻塞心跳与正常广播；记录网页栈余量和 NVS 提交延迟 |
| H06 | 复位或替换 RID、复位飞控、切换飞行器/连接 | boot/session/UID 正确更新；旧帧和旧 HTTP 回调不能恢复就绪 |
| H07 | 向另一节点发送 GOOD ArmStatus；接第二个 QGC | 无法借另一个节点通过检查；活跃报送会话不被静默抢占 |
| H08 | 停 QGC 心跳、冻结服务 ACK、重放/倒退序号 | 心跳 3 秒或 ACK 5 秒后失效；重复/倒退序号不能续期，QGC 的 ACK 年龄随实际最后一次受理持续增长 |
| H09 | 注入鉴权、网络、坏 JSON、身份不匹配、超时 | 解锁前阻止就绪；错误可见、重试退避且最多一个在途请求、服务恢复后自动恢复 |
| H10 | 台架模拟已解锁飞行状态，再注入单独 UOM 故障 | 只有告警、日志、重试；没有 UOM 导致的返航、降落或停机 |
| H11 | 同场景改为真正 RID 故障 | 仍执行原有独立 RID 故障策略，不能套用 UOM 的仅告警规则 |
| H12 | GPS/位置样本冻结、倒退、无效 UTC、操作员位置过期 | 不以重发或接收时间掩盖旧数据；恢复新鲜有效样本后恢复 |
| H13 | 换生产 PX4，保持模拟 UOM 就绪 | 生产飞控拒绝模拟环境；不要靠参数把模拟服务冒充正式环境 |
| H14 | 独立接收端读取无线帧 | 产品码、UIN、位置、高程基准、UTC 与真实输入一致，核验目标接收端互通 |
| H15 | NVS 提交期间断电、单槽损坏、恢复默认 | 旧或新完整有效授权可恢复；不出现半包身份；恢复默认保留 active |

已解锁后 UOM 仅提示，不新增自动解除武装或起飞权限策略。当前协议信任匹配的 QGC 报送状态，不声称防止已控制 QGC 的攻击者伪造 ACK 年龄并递增序号。

H10 应先在台架完成，不要求为此执行真实风险飞行。自动测试不能替代 H05、H14 的物理时序和无线互通量测。

CAN 与标准 MAVLink 的动态 Location/System 发送频率为 5 Hz，静态身份为 1 Hz；默认 MAVLink 配置相比原 1 Hz 动态流增加约 584 B/s。GetSet 单请求超时 600 ms、单次快照上限 6 s、总查询上限 20 s；身份缓存有效期 12 s，回复缓存不会延长实际 CAN 验证时间。System 单独使用“接收小于 1 s、整秒 UTC 有进展小于 2 s”的双窗口，避免 5 Hz 流跨秒抖动误报，同时拒绝冻结 UTC 的重复包；GPS/Location 源样本仍限定 1 s。

## 定位入口

PX4 控制台可查看 `listener rid_identity`、`listener rid_uom_status`、`listener open_drone_id_arm_status`；默认日志已包含前两项。核对 selected node、boot、RID boot、revision、session、hash、ACK 年龄及来源。

RID 查询 `/ajax/activation.json` 与 `/ajax/status.json`；HTTP 收包成功、验签暂存成功、冷启动生效是三个不同阶段。保留 `web_stack_min_free_bytes`、`free_heap_bytes` 和上传期间 CAN 抓包。

QGC 记录 `Vehicle.UOM` 状态，模拟服务日志只输出请求状态，不输出授权令牌和完整遥测。联调问题请同时保留三端日志及触发步骤。

## 重建入口

```sh
cd /home/amovlgf/PX4-Autopilot
make amovlab_flycore_rid -j4
make amovlab_flycore_ridbench -j4
```

不同 PX4/NuttX 目标串行构建，避免共享 NuttX 配置竞争。

```sh
cd /home/amovlgf/ArduRemoteID
make -C RemoteIDModule amov-rid
make -C RemoteIDModule amov-rid-test
```

QGC 本机已有隔离镜像 `amov-m4-qgc:qt612-gcc12` 与 Qt kit `~/.local/amov-m4/Qt`，可用 `bash tools/amov_m4/build_qgc.sh` 增量重建。容器文件位于 `tools/amov_m4/Dockerfile`，Qt 外置；完整构建关闭 GStreamer 视频，RID/UOM 联调不依赖视频。启动入口使用容器运行库，不依赖宿主 Ubuntu 20.04 的旧 Qt。

正式接入前仍需公司公钥和签发服务、经确认的正式 UOM 接口/证书/账号及对应 provider，随后重新完成生产验收。
