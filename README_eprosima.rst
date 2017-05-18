PX4-FastRTPS
============

This is a fork of PX4 Firmware repository to add communication capabilities between a **PX4 Autopilot** (in this README we will talk about **PIXracer**) and a machine running **Fast RTPS** through serial ports using **CDR serialization**, aims to get information from a drone and carry to the DDS world through **Fast RTPS** and put information into the drone from DDS as same manner.

.. image:: doc/1_general-white.png

Automatic code generation
-------------------------

The support for the new functionality added is mainly carried on inside three new (automatic generated) code blocks.

-  It's added a CDR serialization support directly on the original type support of the uORB topic adding a new interface to the code that do this directly. For *sensor_combined.msg* topic looks like this:

.. code-block:: shell

   void serialize_sensor_combined(const sensor_combined_s *input, char *output, uint32_t *length);
   void deserialize_sensor_combined(struct sensor_combined_s *output, char *input);

-  We have the capability under demand of the generation of an application to send and receive through a selected UART the serializated info from several topics (*miroRTPS_client.cpp*).

.. image:: doc/2_trasnmitter-white.png

-  We also have the capacity of generate automatically the support for the other side of the communication, **Fast RTPS** through auto generated *microRTPS_agent.cxx* application and .idl files for demanded topics. For the case of *sensor_combined* topic it's generated *sensor_combined_.idl* file.

.. image:: doc/3_receiver-white.png

-  This covers the entire spectrum of communications.

.. image:: doc/4_both-white.png

The code for extended topic support is generated within the normal PX4 Firmware generation process. The other will be generated under demand calling the script **generate_microRTPS_bridge.py** placed in *Tools* folder, on this way:

.. code-block:: shell

    $ usage: generate_microRTPS_bridge.py [-h] [--send/-s *.msg [*.msg ...]] [--receive/-r *.msg [*.msg ...]]
    $ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg

The argument **--send/-s** means that the application from PX4 side will send these messages, and the argument **--receive/-r** specifies which messages is going to be received. The output appears in the *msgenerated* folder, in this case:

.. code-block:: shell

    $ ls msgenerated/
    microRTPS_agent_CMakeLists.txt
    microRTPS_agent.cxx
    microRTPS_client_CMakeLists.txt
    microRTPS_client.cpp
    sensor_baro_.idl
    sensor_baro_Publisher.cxx
    sensor_baro_Publisher.h
    sensor_combined_.idl
    sensor_combined_Subscriber.cxx
    sensor_combined_Subscriber.h

Installig code
--------------

At this point we need to have installed Fast RTPS to continue. Visit its installation `manual <http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html>`_ for more information.

For automating the code installation we need to run a script. The script creates */path/to/micrortps_agent/install-dir* and *src/example/micrortps_client* if doesn't exist, and install both applications inside:

    $ ./micrortps_install.sh /path/to/micrortps_agent/install-dir /path/to/Fast-RTPS/install-dir/bin/

**CAUTION**: This script erase some files from */path/to/micrortps_agent/install-dir*

PX4 Firmware
------------

On the *PX4* side, it will be used an application running an uORB node. This node could be subscribed to a several internal topics as well as publish under these. The application receive from a internal publishers the messages in a loop, serializes the struct and writes it trough an UART port selected by the user. Also will be reading from the UART port and publish to the internal subscribers.

Steps to use the auto generated application:

-  Uncomment in *cmake/configs/nuttx_px4fmu-v4_default.cmake* file the *#examples/micrortps_client* to compile this appication along the **PX4** firmware:

.. code-block:: shell

    # eProsima app
    examples/micrortps_client

-  Construct and upload the firmware executing:

.. code-block:: shell

   $ make px4fmu-v4_default upload

After uploading the firmware, the application can be launched on *NuttShell* typing its name and passing an available serial port as argument. Using */dev/ttyACM0*
will use the USB port as output. Using */dev/ttyS1* or */dev/ttyS2* will write the output through TELEM1 or TELEM2 ports respectively.

.. code-block:: shell

    > micrortps_client [U [P [L [S]]]]
        U: minimum update time in ms for uORB topics
        P: maximum wait (poll) time in ms for new uORB topic updates
        L: number of loops of the application
        S: sleep time for each loop in us

    > micrortps_client 10 10 1000 2000 #by default

**NOTE**: If the UART port selected is busy, it's possible that Mavlink applications were using them. If it is the case, you can stop Mavlink from NuttShell typing:

.. code-block:: shell

    > mavlink stop-all

Fast RTPS (Raspberry PI application)
------------------------------------

The *Fast RTPS* side will be explained taking a *Raspberry Pi* board to run an application as example.

The application have several functions and possibilities of use: get the sensor data from a system that is using the *PX4 Firmware* (reading the info from the selected UART),
publish this to a *Fast RTPS* environment, write info to the UART from topics that are expected in the *PX4* side with the info even from subscribed messages from *Fast RTPS* side.

Before runnning the application, it is needed to have installed Fast RTPS. Visit it installation `manual <http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html>`_ for more information.

This section explains how create *Fast RTPS* applications using the files generated by **generate_microRTPS_bridge.py** and **fastrtpsgen** (this step performed inside install script) from *Fast RTPS*.

This application allow to launch a publisher that will be using the information coming from the uORB topic in the PX4 side thanks to the autogenerated idl file from the original msg file. The publisher will read data from the UART, deserializes it, and make a Fast RTPS message mapping the attributes from the uORB message. The subscriber simply receives the Fast RTPS messages and print them to the terminal. The subscriber can be launched on the Raspberry Pi or in any another device connected in the same network.

For create the application, compile the code:

.. code-block:: shell

   $ cd /agent/installation/path/
   $ mkdir build && cd build
   $ cmake ..
   $ make

To launch the publisher run:

.. code-block:: shell

    $ ./micrortps_receiver [UART [S]]
      UART: selected UART
      S: sleep time for each loop in us

    $ ./micrortps_receiver /dev/ttyACM0 2000 #by default

Now we can add some code to print some info on the screen, for example:

.. code-block:: shell
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
