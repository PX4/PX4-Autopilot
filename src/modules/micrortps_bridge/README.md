# PX4-FastRTPS bridge


This bridge adds communication capabilities between a **PX4 Autopilot** and a **Fast RTPS** application through serial ports or
UDP sockets using **CDR serialization**. The goal is to provide a DDS (Data Distribution Service) interface to PX4. This interface allows sharing information also with the upcoming release of ROS (Robot Operating System), ROS2.

DDS is an standard from the OMG (Object Management Group) providing a real-time pub/sub middleware widely used in aerospace, defense and IoT applications, while in Robotics has been adopted as the middleware for ROS2.

![alt text](res/1_general-white.png)

## Automatic code generation

The support for the functionality is mainly done within three new (automatically generated) code blocks.

-  CDR serialization support is generated for every uORB topic adding a interfaces what for *sensor_combined.msg* topic looks like this:

  ```sh
  void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
  void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
  ```

-  An application to send and receive the CDR serialized data from several topics through a selected UART or selected UDP ports. This is called the client.

![alt text](res/2_trasnmitter-white.png)

-  The other side of the communication is an application between CDR serialized data and the
ROS2/DDS world (using **Fast RTPS**). An important step in this part is the generation of a idl file for each topic which
is the translation of the correspondent msg file. For the case of  *sensor_combined* topic it's generated the
*sensor_combined_.idl* file from *sensor_combined.msg* file. This application is called the agent.

![alt text](res/3_receiver-white.png)

This covers the entire spectrum of communications.

![alt text](res/4_both-white.png)

These pieces of code are generated within the normal PX4 Firmware generation process. Also can be generated under demand calling the script **generate_microRTPS_bridge.py** placed in *Tools* folder, see section below.

### Generating the client and the agent

**NOTE**: Before continue we need to have installed Fast RTPS. Visit its installation
[manual](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html) for more information.

The generation of the code is performed automatically through the normal process of compilation of the PX4 Firmware. Only we need to specify the topics we want to send and receive. For that purpose we need to add them in the **.cmake** file (*cmake/configs*) correspondent with our target platform in this way:

```cmake
set(config_rtps_send_topics
   sensor_accel
   sensor_baro
   sensor_gyro
   ...
   )

set(config_rtps_receive_topics
   sensor_combined
   telemetry_status
   wind_estimate
   ...
   )
```

The client application will be generated in *build_OURPLATFORM/src/modules/micrortps_bridge/micrortps_client/* folder and the agent will be created in *src/modules/micrortps_bridge/micrortps_agent/* folder.

**NOTE**: In the process of the agent generation we use a Fast RTPS tool called **fastrtpsgen**. If you don't have installed Fast RTPS in the default path you need to specify the directory of installation of fastrtpsgen setting the environment variable **FASTRTPSGEN_DIR** before make executing in this way:

```sh
export FASTRTPSGEN_DIR=/path//to/fastrtps/install/folder/bin
```

Also is possible to generate and install the code for the client and the agent outside the normal process of PX4 generation executing directly the python script
**generate_microRTPS_bridge.py** hosted in *Tools* folder.

  ```text
  $ cd /path/to/PX4/Firmware
  $ python Tools/generate_microRTPS_bridge.py -h
  usage: generate_microRTPS_bridge.py [-h] [-s *.msg [*.msg ...]]
                                      [-r *.msg [*.msg ...]] [-a] [-c]
                                      [-t MSGDIR] [-o AGENTDIR] [-u CLIENTDIR]
                                      [-f FASTRTPSGEN]

  optional arguments:
    -h, --help            show this help message and exit
    -s *.msg [*.msg ...], --send *.msg [*.msg ...]
                          Topics to be sended
    -r *.msg [*.msg ...], --receive *.msg [*.msg ...]
                          Topics to be received
    -a, --agent           Flag for generate the agent, by default is true if -c
                          is not specified
    -c, --client          Flag for generate the client, by default is true if -a
                          is not specified
    -t MSGDIR, --topic-msg-dir MSGDIR
                          Topics message dir, by default msg/
    -o AGENTDIR, --agent-outdir AGENTDIR
                          Agent output dir, by default
                          src/modules/micrortps_bridge/micrortps_agent
    -u CLIENTDIR, --client-outdir CLIENTDIR
                          Client output dir, by default
                          src/modules/micrortps_bridge/micrortps_client
    -f FASTRTPSGEN, --fastrtpsgen-dir FASTRTPSGEN
                          fastrtpsgen installation dir, by default /bin
    --no-delete           Do not delete dir tree output dir(s)
  ```

 - The argument **--send/-s** means that the application from PX4 side will send these messages, and the argument **--receive/-r** specifies which messages is going to be received.

 - The output appears in CLIENTDIR (**-o src/modules/micrortps_bridge/micrortps_client**, by default) and in the AGENTDIR (**-u src/modules/micrortps_bridge/micrortps_agent**, by default). **CAUTION**: This script erase the content of the CLIENTDIR and the AGENTDIR before create new files and folders.

 - If no flag **-a** or **-c** is specified, both the client and the agent will be generated and installed.

 - The **-f** option may be needed if *Fast RTPS* was installed in other path different to default one (*-f /path/to/fastrtps/installation/bin*).

An example of use:

  ```sh
  $ cd /path/to/PX4/Firmware
  $ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
  ```
Checking the correct installation:

  ```sh
  $ tree src/modules/micrortps_bridge/micrortps_agent
  src/modules/micrortps_bridge/micrortps_agent
  ├── build
  ├── CMakeLists.txt
  ├── idl
  │   ├── sensor_baro_.idl
  │   └── sensor_combined_.idl
  ├── microRTPS_agent.cxx
  ├── microRTPS_transport.cxx
  ├── microRTPS_transport.h
  ├── RtpsTopics.cxx
  ├── RtpsTopics.h
  ├── sensor_baro_.cxx
  ├── sensor_baro_.h
  ├── sensor_baro_Publisher.cxx
  ├── sensor_baro_Publisher.h
  ├── sensor_baro_PubSubTypes.cxx
  ├── sensor_baro_PubSubTypes.h
  ├── sensor_combined_.cxx
  ├── sensor_combined_.h
  ├── sensor_combined_PubSubTypes.cxx
  ├── sensor_combined_PubSubTypes.h
  ├── sensor_combined_Subscriber.cxx
  └── sensor_combined_Subscriber.h

  2 directories, 20 files
  ```
  ```sh
  $ tree src/modules/micrortps_bridge/micrortps_client
  src/modules/micrortps_bridge/micrortps_client
  ├── CMakeLists.txt
  ├── microRTPS_client.cpp
  ├── microRTPS_transport.cxx
  └── microRTPS_transport.h

  0 directories, 4 files
  ```

PX4 Firmware: the micro RTPS client
-----------------------------------

On the *PX4* side, it will be used an application running as a uORB node and as a micro RTPS node at same time.
This application as uORB node could be subscribed to a several topics as well as publish under internal uORB topics.
The application receives from a internal publishers the messages, serializes the struct and writes it trough an UART
device or UDP port selected by the user. Also will be reading from the UART device or UDP port and then publish the info
to the internal subscribers.

Steps to use the auto generated application:

-  Check that the line **modules/micrortps_bridge/micrortps_client** and the line **lib/micro-CDR** exist in the .cmake config file for the target platform (*cmake/configs/*). This enables the compilation of the client along the **PX4** firmware. For the *Pixracer* platform we found this in *cmake/configs/nuttx_px4fmu-v4_default.cmake* file and for the *Snapdragon Flight* platform we found it in *cmake/configs/posix_sdflight_default.cmake*:

  ```sh
  set(config_module_list
      ...
      lib/micro-CDR
      ...
      # micro RTPS
      modules/micrortps_bridge/micrortps_client
      ...
      )
  ```
-  Construct and upload the firmware executing, for example:

  ```sh
  # For Pixracer:
  $ make px4fmu-v4_default upload
  ```
  ```sh
  # For Snapdragon Flight:
  $ make eagle_default upload
  ```

After uploading the firmware, the application can be launched typing its name and passing an variable number of arguments as shown bellow:

  ```text
  > micrortps_client start|stop [options]
    -t <transport>          [UART|UDP] Default UART
    -d <device>             UART device. Default /dev/ttyACM0
    -u <update_time_ms>     Time in ms for uORB subscribed topics update. Default 0
    -l <loops>              How many iterations will this program have. -1 for infinite. Default 10000
    -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
    -b <baudrate>           UART device baudrate. Default 460800
    -p <poll_ms>            Time in ms to poll over UART. Default 1ms
    -r <reception port>     UDP port for receiving. Default 2019
    -s <sending port>       UDP port for sending. Default 2020
  ```
  **NOTE**: If we are working with a USB-serial adapter the **-b** option will be ignored and will be working at maximum speed of the link.
  ```sh
  > micrortps_client start #by default -t UART -d /dev/ttyACM0 -u 0 -l 10000 -w 1 -b 460800 -p 1
  ```

**NOTE**: If the UART port selected is busy, it's possible that Mavlink applications were using them. If it is the case, you can stop Mavlink from NuttShell typing:

  ```sh
  > mavlink stop-all
  ```
## Fast RTPS: the micro RTPS agent

The *Fast RTPS* side will be explained taking a *Raspberry Pi* board to run an application as example.

The application has several functions and possibilities of use: get the sensor data from a system that is using the *PX4
Firmware* (obtaining the information from the selected transport: UDP or UART), publish this to a *Fast RTPS* environment
and, in the other direction, to send through the selected transport the information of topics that are expected in the
*PX4* side with the info even from subscribed messages from *Fast RTPS* side.

Before runnning the application, it is needed to have installed Fast RTPS. Visit it installation [manual](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html) for more information.

This section explains how create *Fast RTPS* applications using the files generated by **generate_microRTPS_bridge.py** and **fastrtpsgen** (this step performed inside install script) from *Fast RTPS*.

On the *Fast RTPS* side, it will be used an application running as a Fast RTPS node and as a micro RTPS node at same time. This application allow to launch RTPS publishers and subscribers that will be using the information coming from and sending to uORB topics in the PX4 side thanks to the autogenerated idl file from the original msg file. The publisher will read data from UART/UDP, deserializes it, and make a Fast RTPS message mapping the attributes from the uORB message. The subscriber simply receives the Fast RTPS messages and send in the reverse sequence to the PX4 side. The subscriber can be launched on the Raspberry Pi or in any another device connected in the same network.

To create the application, compile the code:

  ```sh
  $ cd /agent/installation/path/
  $ mkdir build && cd build
  $ cmake ..
  $ make
  ```

**NOTE**: To crosscompiling for Snapdragon Fligth platform you can see this [link](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use).

To launch the publisher run:

  ```text
  $ ./micrortps_agent [options]
    -t <transport>          [UART|UDP] Default UART
    -d <device>             UART device. Default /dev/ttyACM0
    -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms
    -b <baudrate>           UART device baudrate. Default 460800
    -p <poll_ms>            Time in ms to poll over UART. Default 1ms
    -r <reception port>     UDP port for receiving. Default 2019
    -s <sending port>       UDP port for sending. Default 2020
  ```
  **NOTE**: If we are working with a USB-serial adapter the **-b** option will be ignored and will be working at maximum speed of the link.
  ```sh
  $ ./micrortps_agent # by default -t UART -d /dev/ttyACM0 -w 1 -b 460800 -p 1
  ```

Now we can add some code to print some info on the screen, for example:

  ```cpp
  void sensor_combined_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
  {
      // Take data
      if(sub->takeNextData(&msg, &m_info))
      {
          if(m_info.sampleKind == ALIVE)
          {
              cout << "\n\n\n\n\n\n\n\n\n\n";
              cout << "Received sensor_combined data" << endl;
              cout << "=============================" << endl;
              cout << "timestamp: " << msg.timestamp() << endl;
              cout << "gyro_rad: " << msg.gyro_rad().at(0);
              cout << ", " << msg.gyro_rad().at(1);
              cout << ", " << msg.gyro_rad().at(2) << endl;
              cout << "gyro_integral_dt: " << msg.gyro_integral_dt() << endl;
              cout << "accelerometer_timestamp_relative: " << msg.accelerometer_timestamp_relative() << endl;
              cout << "accelerometer_m_s2: " << msg.accelerometer_m_s2().at(0);
              cout << ", " << msg.accelerometer_m_s2().at(1);
              cout << ", " << msg.accelerometer_m_s2().at(2) << endl;
              cout << "accelerometer_integral_dt: " << msg.accelerometer_integral_dt() << endl;
              cout << "magnetometer_timestamp_relative: " << msg.magnetometer_timestamp_relative() << endl;
              cout << "magnetometer_ga: " << msg.magnetometer_ga().at(0);
              cout << ", " << msg.magnetometer_ga().at(1);
              cout << ", " << msg.magnetometer_ga().at(2) << endl;
              cout << "baro_timestamp_relative: " << msg.baro_timestamp_relative() << endl;
              cout << "baro_alt_meter: " << msg.baro_alt_meter() << endl;
              cout << "baro_temp_celcius: " << msg.baro_temp_celcius() << endl;

              // Print your structure data here.
              ++n_msg;
              //std::cout << "Sample received, count=" << n_msg << std::endl;
              has_msg = true;

          }
      }
  }
  ```

**NOTE**: Normally, for UART transport it's necessary set up the UART port in the Raspberry Pi. To enable the serial port available on Raspberry Pi connector:

1. Make sure the userid (default is pi) is a member of the dialout group:

  ```sh
  $ groups pi
  $ sudo usermod -a -G dialout pi
  ```

2. You need to stop the already running on the GPIO serial console:

  ```sh
  $ sudo raspi-config
  ```

Go to *Interfacing options > Serial*, NO to *Would you like a login shell to be accessible over serial?*, valid and reboot.

3. Check UART in kernel:

  ```sh
  $ sudo vi /boot/config.txt
  ```

And enable UART setting *enable_uart=1*.

## Hello world and Throughput test

Please click in [Hello world](hello_world.md) or in [Throughput test](throughput_test.md) to see these couple of operative example of use.

## Graphical example of usage

This flow chart shows graphically how works a bridge for an example of use that sends the topic sensor_combined from a Pixracer to a Raspberry Pi through UART.

![alt text](res/architecture.png)

If all steps has been followed, you should see this output on the subscriber side of Fast RTPS.

![alt text](res/subscriber.png)
