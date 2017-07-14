# Hello world

As a basic example we go to explain how implement a simple use case what sends information of some sensors (*sensor_combined* uORB topic) in the direction PX4-Fast RTPS and receives information as a text message (*log_message* uORB topic) sent in the other direction.

## Creating the code

  ``` sh
  $ cd /path/to/PX4/Firmware
  $ python Tools/generate_microRTPS_bridge.py --send msg/sensor_combined.msg --receive msg/sensor_combined.msg msg/log_message.msg -u src/examples/micrortps_client
  ```
**NOTE**: It may be needed specify other different arguments, as the path to the Fast RTPS *bin* installation folder if it was installed in other path different to default one (*-f /path/to/fastrtps/installation/bin*). For more information, click this [link](README.md#generate-and-installing-the-client-and-the-agent).

That generates and installs the PX4 side of the code (the client) in *src/examples/micrortps_client* and the Fast RPS side (the agent) in *src/modules/micrortps_bridge/micrortps_agent*.

To see the message received in the client one time each second (**src/examples/micrortps_client/microRTPS_client.cpp**), we change the default value of the sleep to 1000 and we will add this *printf* to the code under the *orb_publish* line for *log_message* topic (line 274):

  ```cpp
  ...
  #define SLEEP_MS 1000
  ...
  ...
  ...
  case 36:
  {
      deserialize_log_message(&log_message_data, data_buffer, &microCDRReader);
      if (!log_message_pub)
          log_message_pub = orb_advertise(ORB_ID(log_message), &log_message_data);
      else
          orb_publish(ORB_ID(log_message), log_message_pub, &log_message_data);
      printf("%s\n", log_message_data.text);
      ++received;
  }
  ...
  ```
To enable the compilation of the example client we need to modify the *micrortps_client* line in the cmake of our platform (*cmake/configs*) in that way:

``` cmake
set(config_module_list
  ...
  #src/modules/micrortps_bridge/micrortps_client
  src/examples/micrortps_client
  ...
)
```

Also we need to a add basic CMakeLists.txt like that:

``` cmake
px4_add_module(
	MODULE examples__micrortps_client
	MAIN micrortps_client
	STACK_MAIN 4096
	SRCS
		microRTPS_transport.cxx
		microRTPS_client.cpp
	DEPENDS
		platforms__common
	)
```

 Now, we change the agent in order to send a log message each time we receive a **Fast RTPS message** with the info of the uORB topic *sensor_combined* (in the Fast RTPS world this topic will be called *sensor_combined_PubSubTopic*).

  - In the **src/modules/micrortps_bridge/micrortps_agent/RtpsTopic.cxx** file we will change the *RtpsTopics::getMsg* function to return a *log_message* for each *sensor_combined* with the text "*The temperature is XX.XXXXXXX celsius degrees*":

```cpp
 bool RtpsTopics::getMsg(const char topic_ID, eprosima::fastcdr::Cdr &scdr)
 {
     bool ret = false;
     switch (topic_ID)
     {
         case 58: // sensor_combined
             if (_sensor_combined_sub.hasMsg())
             {
                 sensor_combined_ msg = _sensor_combined_sub.getMsg();
                 log_message_ log_msg = {};
                 std::string message = "The temperature is " + std::to_string(msg.baro_temp_celcius()) + " celsius degrees";
                 std::memcpy(log_msg.text().data(), message.c_str(), message.length());
                 log_msg.serialize(scdr);
                 ret = true;
             }
         break;
         default:
             printf("Unexpected topic ID to getMsg\n");
         break;
     }

     return ret;
 }
 ```

 - In the **src/micrortps_bridge/micrortps_agent/microRTPS_agent.cxx** we will change the topic ID of the received message to the topic ID of the *log_message* (**36**) that is really the topic we are handling:

```cpp
  ...
  if (topics.getMsg(topic_ID, scdr))
  {
      topic_ID = 36;
      length = scdr.getSerializedDataLength();
      if (0 < (length = transport_node->write(topic_ID, scdr.getBufferPointer(), length)))
      {
          total_sent += length;
          ++sent;
      }
  }
  ...
```
## Result

After compiling and launching both the [client](README.md#px4-firmware-the-micro-rtps-client) and the [agent](README.md#fast-rtps-the-micro-rtps-agent) we obtain this kind of messages in the client shell window (showing the message created in the agent with info from temperature sensor in the PX4 side):

```sh
  ...
  The temperature is: 47.779999 celsius
  ...
```
