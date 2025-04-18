# Holybro PM008-CAN Power Module

This PM08 Power Module supplies regulated 5.2V to the flight controller from the battery while providing current consumption and battery voltage measurements via DroneCAN protocol through a 4-pin JST-GH cable to the Autopilot. 

This power module is designed for a vehicle with high power requirements. It comes housed in an aluminum heatsink, protecting the module from external elements and actively dissipating heat generated during operation to ensure durability and optimal performance. With the Hall effect current sensor, it eliminates the need for active cooling, allowing the module to be installed in a small and enclosed environment.

## Feature

- **Supports 2-14S & 200A Cont, 400A Burst**
- **DroneCAN Communication Protocol**
- **Small & Compact Design**
- **Two 5V outputs with an Independent BEC IC and Circuitry**
- **Separated Sensor and BEC Board to Minimize Electrical Noise Interference**
- **Galvanically Isolated Current Sensor IC (AEC-Q100 Grade 1 qualified)**
- **High Accuracy Temperature Sensor for Temperature Monitor**
- **High-Efficiency Aluminum Heat Dissipation Shell**

## Specifications 

- **Processor**: STM32F405RG 168MHz 1024KB Flash  196KB RAM
- **Voltage input**: 7 ~ 60.9V (2S ~ 14S) 
- **Continuous current**:200A
- **Burst current**
  - 400A @ 25℃ 1 sec
  - 1000A @ 25℃ < 1 sec
- **Max current sensing**: 376A
- **Voltage accuracy**: ±0.1V 
- **Current accuracy**: ±5%
- **Temperature accuracy**:±1℃
- **Power port output**: 5.3V/3A each port
- **Protocol**: DroneCAN 
- **Operating temperature**:-25℃~85℃
- **Software-Controlled CAN Termination**
- **Firmware upgrade**: Support
- **Calibration**:  Support

## Interface Type

- **Power & CAN Port**: Molex CLIK-Mate 2mm 6Pin
- **Battery IN/OUT Options**:
  - XT90 Connectors
  - Tinned Wires
  - XT90 & Ring Terminals

## Status LED

- **Flashing quickly continuously**: MCU is in the bootloader stage, waiting for the firmware to be flash
- **Flashing quickly for 3 seconds, and then on for 1 second**: Waiting for CAN connection
- **Flashing slowly (one-second intervals)**: CAN is successfully connected

## Mechanical Spec

- **Size**: 45mm×41mm×26mm (Not Include Cable)
- **Weight**: 185g (Include Cable)

## Where to Buy

[Order from Holybro Store](https://holybro.com/collections/power-modules-pdbs/products/dronecan-pm08-power-module-14s-200a)
