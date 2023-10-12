# Parallel data logging over MAVLink

### Problem
Starting flight logging over mavlink is a slow operation. In case logging is started from ARMING event the log file most likely have several seconds long missing trace period from the takeoff part. This is caused by the protocol that the log is first filled with header and all the static definitions, configuration, boot-log, etc. which are sent using *reliable_transfer*, meaning that every log message/package the px4 logger is sending needs to be acknowleged by the receiver. The round-trip time for this may be long depending on the MAVLink transfer media and the module receiving the log data in the other system behind the Mavlink.

### Solution
To speed up logging startup and reduce that blackout period in the beginning of the log, the parallel data logging option is implemented. The trick here is that the actual uorb data logging is started as soon as possible when logging is started and the static definitions/configs are sent at the same time through another channel. The receiver end reads both streams and store them to two separate files and in the end of logging it combines them into one ulog file by appending the topic data file in the end of static definition data file.

<span style="color:red">
This new protocol is not backward compatible, so BOTH px4 logger and the receiver MUST or MUST NOT implement the parallel logging to make it work!
</span>

### Original protocol
For Logger, there is one **ulog_stream** uorb channel for transfer data to receiver side and another **ulog_stream_ack** for receiveing ack events. First it collects all the static definitions and send them using *reliable_transfer* method. After static defs are sent it starts sending actual dynamic topic data.

Mavlink_uorb module listen to **ulog_stream** topic and depending on FLAGS_NEED_ACK flag in the topic msg it sends either **MAVLINK_MSG_LOGGING_DATA** or **MAVLINK_MSG_LOGGING_DATA_ACKED** msg over mavlink.
If **MAVLINK_MSG_LOGGING_DATA_ACKED** is sent it starts waiting for **MAVLINK_MSG_LOGGING_DATA_ACK** and continue sending only after it receives the ack message. publish it to **ulog_stream_ack**

The receiver reads **MAVLINK_MSG_LOGGING_DATA/_ACKED** messages and store them to ulg file. If **MAVLINK_MSG_LOGGING_DATA_ACKED** is received then **MAVLINK_MSG_LOGGING_DATA_ACK** is sent back to PX4.

```
+----------------------------------------------+
|        Logger                                |
|                                              |
|----------------------------------------------|
| Static data                                  |
| Dyn data                                     |
+----------------------------------------------+
    |                                        ^
    | Publish                                | Subs
    | <ulog_stream>                          | <ulog_stream_ack>
    V                                        |
+----------------------------------------------+
|                Mavlink_ulog                  |
|                                              |
|                                              |
|                                              |
+----------------------------------------------+
    |                                        ^
  Send                                       |
MAVLINK_MSG_LOGGING_DATA                   Recv
MAVLINK_MSG_LOGGING_DATA_ACKED          MAVLINK_MSG_
    |                                 LOGGING_DATA_ACK
    V                                        |
+----------------------------------------------+
|                  Receiver                    |
|----------------------------------------------|
| Static data                                  |
| Dyn data                                     |
+----------------------------------------------+
    |
    |
    |
    V
+------------+
| .ulg file  |
+------------+

```


### Parallel logging protocol
Logger spawns new thread for sending Static definitions data (reliable transfer enabled) and continues to send dynamic topic data at the same time. Static data is published into **ulog_stream_acked** topic and the dynamic data into **ulog_stream** topic. The thread sending dynamic data does not need to wait anything, but continuously sending the data without waiting any ack. The static data sender thread publishes one message at a time and waits for ack until publishing next one.

mavlink_uorb reads both **ulog_stream** and **ulog_stream_acked** streams and sends either **MAVLINK_MSG_LOGGING_DATA** or **MAVLINK_MSG_LOGGING_DATA_ACKED** mavlink msgs accordingly. Also it listens to **MAVLINK_MSG_LOGGING_DATA_ACK** messages and publish to **ulog_stream_ack** if one received.
Sending **MAVLINK_MSG_LOGGING_DATA_ACKED** raises a flag to wait for ack. A **MAVLINK_MSG_LOGGING_DATA_ACK** message with expected sequence number shall be received before next _acked message can be sent, but the **MAVLINK_MSG_LOGGING_DATA** messages are always sent in parallel of that without any blocking.

Receiver listens to both **MAVLINK_MSG_LOGGING_DATA** and **MAVLINK_MSG_LOGGING_DATA_ACKED** messages and write them to separate files accordingly: _DATA into .data file and _DATA_ACKED into .ulg file. For each **MAVLINK_MSG_LOGGING_DATA_ACKED** message it sends back a **MAVLINK_MSG_LOGGING_DATA_ACK** message. When logging is stopped, the receiver append the .data file content into the end of .ulg file and removes the .data file.

```
+----------------------------------------------+
|        Logger                                |
|                                              |
|-----------+       +--------------------------|
| Dyn data  |       |       Static data        |
+----------------------------------------------+
    |                 |                      ^
    | Publish         | Publish              | Subs
    | <ulog_stream>   | <ulog_stream_acked>  | <ulog_stream_ack>
    v                 v                      |
+----------------------------------------------+
|                MAvlink_ulog                  |
|                                              |
|                                              |
|                                              |
+----------------------------------------------+
    |                 |                      ^
MAVLINK_MSG_      MAVLINK_MSG_               |
LOGGING_DATA    LOGGING_DATA_ACKED           |
    |                 |                 MAVLINK_MSG_
    |                 |               LOGGING_DATA_ACK
    v                 v                      |
+----------------------------------------------+
|                  Receiver                    |
|-----------+       +--------------------------|
| Dyn data  |       |       Static data        |
+----------------------------------------------+
    |                           |
    |                           |
    |                           |
    v                           v
+------------+            +------------+
| .data file |            | .ulg file  |
|    DATA    |            |    DEFS    |
+------------+            +------------+
    |                           |
    |  ---- Stop logging----    |
    |                           |
    +-------------------------->+
                                |
                                v
                          +------------+
                          | .ulg file  |
                          |    DEFS    |
                          |    DATA    |
                          +------------+

```
