# Iridium/RockBlock Satellite Communication System

A satellite communication system can be used to provide long range high latency link between a ground station and a vehicle.

This topic describes how to set up a system that uses RockBlock as the service provider for the Iridium SBD Satellite Communication System.
Given good signal quality, users can expect a latency between 10 to 15 seconds.

## Overview

The following components are needed for the satellite communication link:

- A [RockBlock 9603 Iridium Satellite Modem](https://www.iridium.com/products/rock-seven-rockblock-9603/) module connected to a Pixhawk flashed with the PX4 Autopilot.
- A message relay server running Ubuntu Linux.
- A ground station computer running _QGroundControl_ on Ubuntu Linux

The full system architecture is shown below:

![Architecture](../../assets/satcom/architecture.jpg)

::: info
The setup was tested with the current release of _QGroundControl_ running on Ubuntu 14.04 and 16.04.

- It may be possible to run the system on other ground stations and operating systems, but this has not been tested (and is not guaranteed to work).
- The [RockBlock MK2](https://www.groundcontrol.com/us/product/rockblock-9602-satellite-modem/) module can also be used.
  The RockBlock 9603 module is recommended because it is smaller and lighter, while providing the same functionality.

:::

## Costs

The UK link running cost consists of a line rental and per message cost:

- Each module needs to be activated which costs £10.00 per month
- Each message transmitted over the system costs one _credit_ per 50 bytes.
  Bundles of credits can be bought from RockBlock for £0.04-£0.11 per credit, depending on the bundle size.

Refer to the [RockBlock Documentation](https://docs.rockblock.rock7.com/docs) for a detailed explanation of the modules, running costs and _RockBlock_ in general.

## Vehicle Setup

### Wiring

Connect the RockBlock module to a serial port of the Pixhawk.
Due to the power requirements of the module it can only be powered over a high-power serial port as a maximum of 0.5 A at 5 V are required.
If none is available/free then another power source which has the same ground level as the Pixhawk and can provide required power has to be setup.
The details of the [connectors](https://docs.rockblock.rock7.com/docs/connectors) and the [power requirements](https://docs.rockblock.rock7.com/docs/power-supply) can be found in the RockBlock documentation.

### Module

The module can either use the internal antenna or an external one connected to the SMA connector.
To [switch between the two antennas modes](https://docs.rockblock.rock7.com/docs/switching-rockblock-9603-antenna-mode) the position of a small RF link cable needs to changed.
If an external antenna is used always make sure that the antenna is connected to the module before powering it up to avoid damage to the module.

The default baud rate of the module is 19200. However, the PX4 _iridiumsbd_ driver requires a baud rate of 115200 so it needs to be changed using the [AT commands](https://www.groundcontrol.com/en/wp-content/uploads/2022/02/IRDM_ISU_ATCommandReferenceMAN0009_Rev2.0_ATCOMM_Oct2012.pdf).

1. Connect to the module with using a 19200/8-N-1 setting and check if the communication is working using the command: `AT`.
   The response should be: `OK`.
1. Change the baud rate:

   ```
   AT+IPR=9
   ```

1. Reconnect to the model now with a 115200/8-N-1 setting and save the configuration using:

   ```
   AT&W0
   ```

The module is now ready to be used with PX4.

### Software

[Configure the serial port](../peripherals/serial_configuration.md) on which the RockBlock module will run using [ISBD_CONFIG](../advanced_config/parameter_reference.md#ISBD_CONFIG).
There is no need to set the baud rate for the port, as this is configured by the driver.

::: info
If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware):

```
drivers/telemetry/iridiumsbd
```

:::

## RockBlock Setup

When buying the first module on RockBlock an user account needs to be created in a first step.

Log in to the [account](https://rockblock.rock7.com/Operations) and register the RockBlock module under the `My RockBLOCKs`.
Activate the line rental for the module and make sure that enough credits for the expected flight duration are available on the account.
When using the default settings one message per minute is sent from the vehicle to the ground station.

Set up a delivery group for the message relay server and add the module to that delivery group:

![Delivery Groups](../../assets/satcom/deliverygroup.png)

## Relay Server Setup

The relay server should be run on either Ubuntu 16.04 or 14.04 OS.

1. The server working as a message relay should have a static IP address and two publicly accessible, open, TCP ports:

   - `5672` for the _RabbitMQ_ message broker (can be changed in the _rabbitmq_ settings)
   - `45679` for the HTTP POST interface (can be changed in the **relay.cfg** file)

1. Install the required python modules:

   ```sh
   sudo pip install pika tornado future
   ```

1. Install the `rabbitmq` message broker:

   ```sh
   sudo apt install rabbitmq-server
   ```

1. Configure the broker's credentials (change PWD to your preferred password):

   ```sh
   sudo rabbitmqctl add_user iridiumsbd PWD
   sudo rabbitmqctl set_permissions iridiumsbd ".*" ".*" ".*"
   ```

1. Clone the [SatComInfrastructure](https://github.com/acfloria/SatComInfrastructure.git) repository:

   ```sh
   git clone https://github.com/acfloria/SatComInfrastructure.git
   ```

1. Go to the location of the _SatComInfrastructure_ repo and configure the broker's queues:

   ```sh
   ./setup_rabbit.py localhost iridiumsbd PWD
   ```

1. Verify the setup:

   ```sh
   sudo rabbitmqctl list_queues
   ```

   This should give you a list of 4 queues: `MO`, `MO_LOG`, `MT`, `MT_LOG`

1. Edit the `relay.cfg` configuration file to reflect your settings.
1. Start the relay script in the detached mode:

   ```sh
   screen -dm bash -c 'cd SatcomInfrastructure/; ./relay.py
   ```

Other instructions include:

- Detach from the screen:

  ```sh
  ctrl+a d
  ```

- Kill execution of the script:

  ```sh
  ctrl+a :quit
  ```

- Reattach to the screen:

  ```sh
  screen -dr
  ```

## Ground Station Computer

To setup the ground station:

1. Install the required python modules:

   ```sh
   sudo pip install pika tornado future
   ```

1. Clone the SatComInfrastructure repository:

   ```sh
   git clone https://github.com/acfloria/SatComInfrastructure.git
   ```

1. Edit the **udp2rabbit.cfg** configuration file to reflect your settings.
1. [Install _QGroundControl_](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (daily build).
1. Add a UDP connection in QGC with the parameters:

   - Listening port: 10000
   - Target hosts: 127.0.0.1:10001
   - High Latency: checked

   ![High Latency Link Settings](../../assets/satcom/linksettings.png)

### Verification

1. Open a terminal on the ground station computer and change to the location of the _SatComInfrastructure_ repository.
   Then start the **udp2rabbit.py** script:

   ```sh
   ./udp2rabbit.py
   ```

1. Send a test message from [RockBlock Account](https://rockblock.rock7.com/Operations) to the created delivery group in the `Test Delivery Groups` tab.

If in the terminal where the `udp2rabbit.py` script is running within a couple of seconds the acknowledge for a message can be observed, then the RockBlock delivery group, the relay server and the udp2rabbit script are set up correctly:

![udp2rabbit message acknowledge](../../assets/satcom/verification.png)

## Running the System

1. Start _QGroundControl_.
   Manually connect the high latency link first, then the regular telemetry link:

   ![Connect the High Latency link](../../assets/satcom/linkconnect.png)

1. Open a terminal on the ground station computer and change to the location of the _SatComInfrastructure_ repository.
   Then start the **udp2rabbit.py** script:

   ```sh
   ./udp2rabbit.py
   ```

1. Power up the vehicle.
1. Wait until the first `HIGH_LATENCY2` message is received on QGC.
   This can be checked either using the _MAVLink Inspector_ widget or on the toolbar with the _LinkIndicator_.
   If more than one link is connected to the active vehicle the _LinkIndicator_ shows all of them by clicking on the name of the shown link:

   ![Link Toolbar](../../assets/satcom/linkindicator.jpg)

   The link indicator always shows the name of the priority link.

1. The satellite communication system is now ready to use.
   The priority link, which is the link over which commands are send, is determined the following ways:

   - If no link is commanded by the user a regular radio telemetry link is preferred over the high latency link.
   - The autopilot and QGC will fall back from the regular radio telemetry to the high latency link if the vehicle is armed and the radio telemetry link is lost (no MAVLink messages received for a certain time).
     As soon as the radio telemetry link is regained QGC and the autopilot will switch back to it.
   - The user can select a priority link over the `LinkIndicator` on the toolbar.
     This link is kept as the priority link as long as this link is active or the user selects another priority link:

     ![Prioritylink Selection](../../assets/satcom/linkselection.png)

## Troubleshooting

- Satellite communication messages from the airplane are received but no commands can be transmitted (the vehicle does not react)
  - Check the settings of the relay server and make sure that they are correct, especially the IMEI.
- No satellite communication messages from the airplane arrive on the ground station:

  - Check using the system console if the _iridiumsbd_ driver started and if it did that a signal from any satellite is received by the module:

    ```sh
    iridiumsbd status
    ```

  - Make sure using the verification steps from above that the relay server, the delivery group and the `udp2rabbit.py` script are set up correctly.
  - Check if the link is connected and that its settings are correct.

- The IridiumSBD driver does not start:

  - Reboot the vehicle.
    If that helps increase the sleep time in the `extras.txt` before the driver is started.
    If that does not help make sure that the Pixhawk and the module have the same ground level. Confirm also that the baudrate of the module is set to 115200.

- A first message is received on the ground but as soon as the vehicle is flying no message can be transmitted or the latency is significantly larger (in the order of minutes)
  - Check the signal quality after the flight.
    If it is decreasing during the flight and you are using the internal antenna consider using an external antenna.
    If you are already using the external antenna try moving the antenna as far away as possible from any electronics or anything which might disturb the signal.
    Also make sure that the antenna is not damaged.
