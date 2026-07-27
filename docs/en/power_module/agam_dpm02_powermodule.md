# Agam DPM02 Power Module

The Agam DPM02 is a digital power module with an integrated Power Distribution Board (PDB) designed for PX4-compatible flight controllers. It provides regulated 5 V power to the flight controller while measuring battery voltage and current through an I2C interface for battery monitoring and power management.

The integrated PDB enables battery power distribution to ESCs, motors, and onboard peripherals.

![Agam DPM02](../../../assets/hardware/power_module/agam_robotics_dpm/agam_dpm02_powermodule.png)

## Where to Buy

Order this module from:

- [Agam DPM02 Power Module + PDB](https://www.agamrobotics.com/product-page/dpm02-digital-power-module-pdb)

## Hardware Specifications

- Integrated Digital Power Module and Power Distribution Board
- Regulated 5 V power output
- Digital voltage and current measurement
- I2C communication interface
- XT60 battery connector
- XT30 motor/ESC outputs
- Molex interface connector
- Compatible with PX4-based flight controllers
- Supports up to 12S LiPo batteries
- Made in India

### Electrical Specifications

- Maximum Input Voltage
  - 60 V

- Rated Current
  - 100 A

- Output Voltage
  - 5 V regulated output

- Maximum Output Current
  - 3 A

- Communication Interface
  - I2C

- Battery Support
  - Up to 12S LiPo battery

- Capacitor
  - 270 µF, 63 V electrolytic capacitor

- Weight
  - 70 g

## Hardware Setup

The DPM02 should be installed between the battery and the flight controller.

1. Connect the battery to the XT60 input connector.
2. Connect the XT30 outputs to the ESCs or motors.
3. Connect the Molex interface cable to the flight controller.
4. Verify all connections before powering the system.

The integrated PDB distributes battery power to ESCs and onboard peripherals.

This module is suitable for quadcopter configurations and lower motor-count platforms.

## PX4 Configuration

Agam Autopilot V6X-RT supports Digital Power Modules such as the DPM02.

In PX4 v1.15 or later, the module is automatically detected.

Enable the following parameter:

- `SENS_EN_INA226` (enabled by default)

No current divider or voltage divider configuration is required in the Battery Configuration settings, unlike analog power modules.

The default calibration values provide measurement accuracy within ±5%.

1. Open QGroundControl.
2. Navigate to **Vehicle Setup > Power**.
3. Verify the battery voltage and current measurements.
4. Confirm that the `SENS_EN_INA226` parameter is enabled.
5. Ensure that the DPM02 is correctly detected by PX4.

## Connector

The DPM02 uses:

- XT60 battery connector
- XT30 ESC outputs
- Molex interface connector

The Molex connector provides power and monitoring signals between the power module and the flight controller.

## Package Contents

- DPM02 Power Module + PDB
- XT60 battery connector
- XT30 ESC connectors
- Molex interface cable
- 270 µF capacitor

## Applications

- Quadcopter UAVs
- Multirotor platforms
- Industrial drones
- Research and educational UAVs
- PX4-based flight controllers

## See Also

- [Agam Robotics](https://www.agamrobotics.com/)
- [Agam DPM02 Product Page](https://www.agamrobotics.com/product-page/dpm02-digital-power-module-pdb)
