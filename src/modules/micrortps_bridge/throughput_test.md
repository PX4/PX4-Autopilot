# Throughput test

This a simple test to measure the throughput of the bridge. Sending an receiving messages of the 256 bytes at the same time as fast as we can.

We will create a new message for this test called **throughput_256.msg**, the content look like this:

  ``` text
  uint8[256] data
  ```
We will create it next to the other *.msg* files in the folder *msg*:

  ``` sh
  $ cd /path/to/PX4/Firmware/msg
  $ echo uint8[256] data > throughput_256.msg
  ```
Now, we register the new message adding it to the list of messages in the *msg/CMakeLists.txt* file:

  ``` cmake
  ...
  wind_estimate.msg
  throughput_256.msg
  )
  ...
  ```
And giving it a topic id, adding a line in the *Tools/message_id.py* script:

  ``` python
  ...
      'wind_estimate': 94,
      'throughput_256': 95,
  }
  ...
  ```

## Creating the code

  ``` sh
  $ cd /path/to/PX4/Firmware
  $ python Tools/generate_microRTPS_bridge.py --send msg/throughput_256.msg --receive msg/throughput_256.msg
  ```

That generate and install the PX4 side of the code (the client) in *src/examples/micrortps_client* and the Fast RPS side (the agent) in *micrortps_agent*.

Now, we have to modify the client to send a *throughput_256* message each time since nobody publish under this topic. In the file **src/examples/micrortps_client/microRTPS_client.cpp** inside the *send* function the loop should look like this:

  ``` cpp
  ...
  while (!_should_exit_task)
  {
      //bool updated;
      //orb_check(fds[0], &updated);
      //if (updated)
      {
          // obtained data for the file descriptor
          struct throughput_256_s data = {};
          // copy raw data into local buffer
          //orb_copy(ORB_ID(throughput_256), fds[0], &data);
          data.data[0] = loop%256;
          serialize_throughput_256(&data, data_buffer, &length, &microCDRWriter);
          if (0 < (read = transport_node->write((char)95, data_buffer, length)))
          {
              total_sent += read;
              ++sent;
          }
      }

      usleep(_options.sleep_ms*1000);
      ++loop;
  }
  ...
  ```
## Result

After compile and launch both the [client](README.md#px4-firmware-the-micro-rtps-client) and the [agent](README.md#fast-rtps-the-micro-rtps-agent) we obtain a measure of throughput for our particular conditions. For a Pixracer and a ordinary PC running Ubuntu 16.04 in the client shell window we obtain:

  ```sh
  SENT:     13255 messages in 13255 LOOPS, 3512575 bytes in 30.994 seconds - 113.33KB/s
  RECEIVED: 13251 messages in 10000 LOOPS, 3511515 bytes in 30.994 seconds - 113.30KB/s
  ```
