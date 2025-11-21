# Frequently Asked Questions

## Build Errors

### Flash Overflow

The amount of code that can be loaded onto a board is limited by the amount of flash memory it has.
When adding additional modules or code its possible that the addition exceeds the flash memory.
This will result in a "flash overflow". The upstream version will always build, but depending on what a developer adds it might overflow locally.

```sh
region `flash' overflowed by 12456 bytes
```

To remedy it, either use more recent hardware or remove modules from the build which are not essential to your use case.
The configuration is stored in **/PX4-Autopilot/boards/px4** (e.g. [PX4-Autopilot/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)).
To remove a module, just comment it out:

```cmake
#tune_control
```

#### Identifying large memory consumers

The command below will list the largest static allocations:

```sh
arm-none-eabi-nm --size-sort --print-size --radix=dec build/px4_fmu-v5_default/px4_fmu-v5_default.elf | grep " [bBdD] "
```

## USB Errors

### The upload never succeeds

On Ubuntu, uninstall the modem manager:

```sh
sudo apt-get remove modemmanager
```
