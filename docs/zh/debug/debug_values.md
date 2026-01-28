# 发送和接收调试值

在软件开发过程中，输出单个重要数字通常是必要的。
This is where the generic `NAMED_VALUE_FLOAT`, `DEBUG` and `DEBUG_VECT` packets of MAVLink come in.

## 在 MAVLink 调试消息和 uORB 主题之间进行映射

MAVLink调试消息转换为/自 uORB 主题。
为了发送或接收 MAVLink 调试消息，您必须分别发布或订阅相应的主题。
下面是一个表，其中总结了 MAVLink 调试消息和 uORB 主题之间的映射：

| MAVLink 消息                                                  | uORB topic                                                |
| ----------------------------------------------------------- | --------------------------------------------------------- |
| NAMED_VALUE_FLOAT | debug_key_value |
| DEBUG                                                       | debug_value                          |
| DEBUG_VECT                             | debug_vect                           |

## 教程：发送字符串/浮点配对

This tutorial shows how to send the MAVLink message `NAMED_VALUE_FLOAT` using the associated uORB topic `debug_key_value`.

本教程的代码可在此处找到：

- [Debug Tutorial Code](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp)
- [Enable the tutorial app](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board) by ensuring the MAVLink debug app (**CONFIG_EXAMPLES_PX4_MAVLINK_DEBUG**) is in the config of your board and set set to 'y'.

设置调试发布所需的只是此代码段。
首先添加头文件：

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
```

然后广播调试值主题（一个针对不同发布名称的广播就足够了）。
把这个放在你的主循环前面：

```C
/* advertise debug value */
struct debug_key_value_s dbg;
strncpy(dbg.key, "velx", sizeof(dbg.key));
dbg.value = 0.0f;
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

而发送主循环更简单：

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

:::warning
Multiple debug messages must have enough time between their respective publishings for Mavlink to process them.
This means that either the code must wait between publishing multiple debug messages, or alternate the messages on each function call iteration.
:::

The result in QGroundControl then looks like this on the real-time plot:

![QGC debugvalue plot](../../assets/gcs/qgc-debugval-plot.jpg)

## 教程：发送字符串/浮点配对

The following code snippets show how to receive the `velx` debug variable that was sent in the previous tutorial.

First, subscribe to the topic `debug_key_value`:

```C
#include <poll.h>
#include <uORB/topics/debug_key_value.h>

int debug_sub_fd = orb_subscribe(ORB_ID(debug_key_value));
[...]
```

当 <code>debug_key_value</code> 主题上有新消息可用时，不要忘记根据其键属性对其进行筛选，以便放弃键与 <code>velx</code> 不同的消息：

```C
[...]
/* one could wait for multiple topics with this technique, just using one here */
px4_pollfd_struct_t fds[] = {
    { .fd = debug_sub_fd,   .events = POLLIN },
};

while (true) {
    /* wait for debug_key_value for 1000 ms (1 second) */
    int poll_ret = px4_poll(fds, 1, 1000);

    [...]
```

When a new message is available on the `debug_key_value` topic, do not forget to filter it based on its key attribute in order to discard the messages with key different than `velx`:

```C
    [...]
    if (fds[0].revents & POLLIN) {
        /* obtained data for the first file descriptor */
        struct debug_key_value_s dbg;

        /* copy data into local buffer */
        orb_copy(ORB_ID(debug_key_value), debug_sub_fd, &dbg);

        /* filter message based on its key attribute */
        if (strcmp(_sub_debug_vect.get().key, "velx") == 0) {
            PX4_INFO("velx:\t%8.4f", dbg.value);
        }
    }
}
```
