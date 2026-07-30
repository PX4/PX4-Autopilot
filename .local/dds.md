# Subscribe to a uORB topic via DDS from Python

## 1. Expose the topic (firmware side)

Check if it's already listed:
```bash
grep <topic_name> src/modules/uxrce_dds_client/dds_topics.yaml
```

If missing, add it under `publications:` (for `/fmu/out/*`):
```yaml
  - topic: /fmu/out/<topic_name>
    type: px4_msgs::msg::<MsgType>
```

Notes:
- `<MsgType>` is the underlying struct, not necessarily the topic name — some
  topics share one message via a `# TOPICS ...` line in the `.msg` file (e.g.
  all `estimator_aid_src_*` topics use `EstimatorAidSource1d`/`3d`). Check
  `msg/*.msg` for a `# TOPICS` comment before assuming a 1:1 name match.
- No new serializer code needed — ucdr (de)serializers are generated for
  every uORB message at build time regardless of this yaml.
- `dds_topics.h.em` asserts `sizeof(struct) <= 512` bytes; bump
  `max_topic_size` there if a topic is bigger.
- Rebuild firmware after editing the yaml.

## 2. Run the agent (separate process, not part of your Python app)

```bash
MicroXRCEAgent udp4 -p 8888
```
This is a standalone binary that bridges the PX4 client to DDS. Your Python
process never instantiates or owns it — it's just another participant on
the DDS network, discovered independently.

## 3. Get the message schema on the Python side

You need `<MsgType>` available as a ROS 2 message, from the `px4_msgs`
package:
```bash
pip show px4_msgs   # or: check it's built in your ROS 2 workspace (colcon build)
```
- If `<MsgType>` already exists there, matching your firmware's PX4 version,
  you're done — no schema work.
- If it doesn't exist, add a `.msg` mirroring the firmware's field
  layout/order exactly (CDR deserialization is layout-sensitive), then
  `colcon build` that package.
- Keep firmware and `px4_msgs` checkouts on matching PX4 versions —
  mismatched `MESSAGE_VERSION` silently breaks deserialization.

## 4. Subscribe

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import <MsgType>

rclpy.init()
node = Node('uorb_listener')
node.create_subscription(
    <MsgType>,
    '/fmu/out/<topic_name>',
    lambda msg: print(msg),
    10,  # QoS depth; PX4 topics are typically best-effort
)
rclpy.spin(node)
```

## Fallback if firmware can't be rebuilt

DDS mapping is closed for that topic without a rebuild — no runtime
registration path exists for arbitrary uORB→DDS exposure. Options:
MAVLink (if the field already rides an existing message), or live ULog
streaming (self-describing, decodes any topic already in the logger config
without knowing its schema in advance).

---
