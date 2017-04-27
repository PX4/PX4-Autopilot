PX4-FastRTPS
============

This is a fork of PX4 Firmware repository to add communication capabilities between a **PX4 Autopilot** (in this README we will talk about **PIXracer**) and a machine running **Fast RTPS** through serial ports using **CDR serialization**, aims to get information from a drone and carry to the DDS world through **Fast RTPS**.

.. image:: doc/1_general-white.png

Automatic code generation
-------------------------

The support for the new functionality added is mainly carried on inside three new (automatic generated) code blocks.

-  It's added a CDR serialization support directly on the original type support of the uORB topic adding a new interface to the code that do this directly. For *sensor_combined.msg* topic looks like this:

.. code-block:: shell

   void serialize_sensor_combined(const sensor_combined_s *input, char *output, uint32_t *length);
   void deserialize_sensor_combined(struct sensor_combined_s *output, char *input);

-  We have the capability under demand of the generation of an application to send and receive through a selected UART the serializated info from several topics (*general_uRTPS_UART_transmitter.cpp*).

.. image:: doc/2_trasnmitter-white.png

-  Now we also have the capacity of generate automatically the support for the other side of the communication, **Fast RTPS** through auto generated *general_uRTPS_UART_PubSubMain.cxx* application and .idl files for demanded topics. For the case of *sensor_combined* topic it's generated *sensor_combined_.idl* file.

.. image:: doc/3_receiver-white.png

-  This covers the entire spectrum of communications.

.. image:: doc/4_both-white.png

The code for extended topic support is generated within the normal PX4 Firmware generation process. The other will be generated under demand calling the new script **generate_microRTPS_support_general.py** placed in *Tools* folder, on this way:

.. code-block:: shell

    $ python Tools/generate_microRTPS_support_general.py [messages...]
    $ python Tools/generate_microRTPS_support_general.py msg/vehicle_status.msg msg/sensor_combined.msg

The output appear in the *msgenerated* folder, for this case:

.. code-block:: shell

    $ ls msgenerated/
    general_transmitter_CMakeLists.txt
    general_uRTPS_UART_PubSubMain.cxx
    general_uRTPS_UART_transmitter.cpp
    sensor_combined_.idl
    sensor_combined_Publisher.cxx
    sensor_combined_Publisher.h
    sensor_combined_Subscriber.cxx
    sensor_combined_Subscriber.h
    vehicle_status_.idl
    vehicle_status_Publisher.cxx
    vehicle_status_Publisher.h
    vehicle_status_Subscriber.cxx
    vehicle_status_Subscriber.h



PX4 Firmware
------------

On the *PX4* side, it will be used an application running an uORB node. This node could be subscribed to a several internal topics as well as publish under these. The application receive from a internal publishers the messages in a loop, serializes the struct and writes it trough an UART port selected by the user. Also will be reading from the UART port and publish to the internal subscribers.

Steps to use the auto generated application:

-  Uncomment in *cmake/configs/nuttx_px4fmu-v4_default.cmake* file the *#examples/micrortps_transmitter* to compile this appication along the **PX4** firmware:

.. code-block:: shell

    # eProsima app
    examples/micrortps_transmitter

-  Create a folder in *src/examples* and copy the above generated application inside:

.. code-block:: shell

   $ mkdir src/examples/micrortps_transmitter
   $ cp msgenerated/general_uRTPS_UART_transmitter.cpp src/examples/micrortps_transmitter

-  Also copy and rename the *CMakeList.txt* and the *UART_Node* class (that give support for serial communication) to the example folder:

.. code-block:: shell

   $ cp msgenerated/general_transmitter_CMakeLists.txt src/examples/micrortps_transmitter/CMakeLists.txt
   $ cp msg/templates/urtps/UART_node.* src/examples/micrortps_transmitter/

-  Construct and upload the firmware executing:

.. code-block:: shell

   $ make px4fmu-v4_default upload

After uploading the firmware, the application can be launched on *NuttShell* typing its name and passing an available serial port as argument. Using */dev/ttyACM0*
will use the USB port as output. Using */dev/ttyS1* or */dev/ttyS2* will write the output trough TELEM1 or TELEM2 ports respectively.

.. code-block:: shell

    > general_trans /dev/ttyACM0  #or /dev/ttySn

**NOTE**: If the UART port selected is busy, it's possible that Mavlink applications were using them. If it is the case, you can stop Mavlink from NuttShell typing:

.. code-block:: shell

    > mavlink stop-all

Fast RTPS (Raspberry PI application)
------------------------------------

The *Fast RTPS* side will be explained taking a *Raspberry Pi* board to run an application as example.

The application have several functions and possibilities of use: get the sensor data from a system that is using the *PX4 Firmware* (reading the info from the selected UART),
publish this to a *Fast RTPS* environment, write info to the UART from topics that are expected in the *PX4* side with the info even from subscribed messages from *Fast RTPS* side.

Before runnning the application, it is needed to have installed Fast RTPS. Visit it installation `manual <http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html>`_ for more information.

This section explains how create *Fast RTPS* applications using the files generated by **generate_microRTPS_support_general.py** and **fastrtpsgen** from *Fast RTPS*.

This application allow to launch a publisher that will be using the information coming from the uORB topic in the PX4 side thanks to the autogenerated idl file from the original msg file. The publisher will read data from the UART, deserializes it, and make a Fast RTPS message mapping the attributes from the uORB message. The subscriber simply receives the Fast RTPS messages and print them to the terminal. The subscriber can be launched on the Raspberry Pi or in any another device connected in the same network.

For create the application:

-  Create a folder and copy the generated idl files in this way:

.. code-block:: shell

    $ mkdir my_app && cd my_app
    $ cp /path/to/Firmware/msgenerated/*.idl .

-  Generate the base application with *fastrtpsgen* and remove unused code:

.. code-block:: shell

    $ /path/to/Fast-RTPS/fastrtpsgen/scripts/fastrtpsgen -example x64Linux2.6gcc *.idl
    $ rm *PubSubMain.cxx

-  Copy the generated code from *generate_microRTPS_support_general.py*, *the UART_node* class and *CMakeLists.txt* in this way:

.. code-block:: shell

    $ cp msgenerated/general_transmitter_CMakeLists.txt CMakeLists.txt
    $ cp /path/to/Firmware/msgenerated/general_uRTPS_UART_PubSubMain.cxx .
    $ cp msg/templates/urtps/UART_node.* .
    $ cp /path/to/Firmware/msgenerated/*Publisher.* .
    $ cp /path/to/Firmware/msgenerated/*Subscriber.* .

Now we can add some code to print some info on the screen, for example:

.. code-block:: shell

   void sensor_combined_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
   {
         // Take data
         sensor_combined_ sensor_data;

         if(sub->takeNextData(&sensor_data, &m_info))
         {
            if(m_info.sampleKind == ALIVE)
            {
                  cout << "\n\n\n\n\n\n\n\n\n\n";
                  cout << "Received sensor_combined data" << endl;
                  cout << "=============================" << endl;
                  cout << "timestamp: " << sensor_data.timestamp() << endl;
                  cout << "gyro_rad: " << sensor_data.gyro_rad().at(0);
                  cout << ", " << sensor_data.gyro_rad().at(1);
                  cout << ", " << sensor_data.gyro_rad().at(2) << endl;
                  cout << "gyro_integral_dt: " << sensor_data.gyro_integral_dt() << endl;
                  cout << "accelerometer_timestamp_relative: " << sensor_data.accelerometer_timestamp_relative() << endl;
                  cout << "accelerometer_m_s2: " << sensor_data.accelerometer_m_s2().at(0);
                  cout << ", " << sensor_data.accelerometer_m_s2().at(1);
                  cout << ", " << sensor_data.accelerometer_m_s2().at(2) << endl;
                  cout << "accelerometer_integral_dt: " << sensor_data.accelerometer_integral_dt() << endl;
                  cout << "magnetometer_timestamp_relative: " << sensor_data.magnetometer_timestamp_relative() << endl;
                  cout << "magnetometer_ga: " << sensor_data.magnetometer_ga().at(0);
                  cout << ", " << sensor_data.magnetometer_ga().at(1);
                  cout << ", " << sensor_data.magnetometer_ga().at(2) << endl;
                  cout << "baro_timestamp_relative: " << sensor_data.baro_timestamp_relative() << endl;
                  cout << "baro_alt_meter: " << sensor_data.baro_alt_meter() << endl;
                  cout << "baro_temp_celcius: " << sensor_data.baro_temp_celcius() << endl;
            }
         }
   }

- Finally we compile the code:

.. code-block:: shell

   $ mkdir build && cd build
   $ cmake ..
   $ make


Now, to launch the publisher run:

.. code-block:: shell

    $ ./micrortps_receiver /dev/ttyACM0 #or the selected UART

**NOTE**: Normally, it's necessary set up the UART port in the Raspberry Pi. To enable the serial port available on Raspberry Pi connector:

1. Make sure the userid (default is pi) is a member of the dialout group:

.. code-block:: shell

    $ groups pi
    $ sudo usermod -a -G dialout pi

2. You need to stop the already running on the GPIO serial console:

.. code-block:: shell

    $ sudo raspi-config

Go to *Interfacing options > Serial*, NO to *Would you like a login shell to be accessible over serial?*, valid and reboot.

3. Check UART in kernel:

.. code-block:: shell

   $ sudo vi /boot/config.txt

And enable UART setting *enable_uart=1*.

Result
------

The entire application will follow this flow chart:

.. image:: doc/architecture.png

If all steps has been followed, you should see this output on the subscriber side of Fast RTPS.

.. image:: doc/subscriber.png

A video of this final process as demostration is available on `https://youtu.be/NF65EPD-6aY <https://youtu.be/NF65EPD-6aY>`_

Testing
-------

Some tests to measure the throughput performed with a PixRacer (microcontroller STM32F42X - 168MHz) in the PX4 side and a laptop in the Fast RTPS side:

1. **Loop latency for one topic - 11ms**: For one topic (sensor_combined) is measured over all the process, the time between the POLLIN event reception over the *px4_pollfd_struct_t* for the subscribed topic and the deserialization of the same message when returns back through the UART.

2. **Transmission speed for one topic - 18kB/s**: For one topic (sensor_combined 72 bytes) is measured the reception speed in the Fast RTPS side, taking in to account with the delay from the performing of whole process in this side, from reception to the dispatch back of the info from the Fast RTPS twin topic.

3. **Transmission speed for ten topics - 32kB/s**: For ten topics (several sizes and frequencys) simultaneously and measured in the reception in the Fast RTPS side, as above.

+-------------------------------+--------+-------------------------+
|                               |        |        Fast RTPS        |
+              TEST             + TOPICS +----------+------+-------+
|                               |        | Received | Sent |  kB/s |
+-------------------------------+--------+----------+------+-------+
|  general_trans 0 0 1000 2000  |    1   |   7959   | 7959 | 17.95 |
+-------------------------------+--------+----------+------+-------+
| general_trans 10 10 1000 1000 |   10   |   1692   | 1692 | 31.87 |
+-------------------------------+--------+----------+------+-------+
