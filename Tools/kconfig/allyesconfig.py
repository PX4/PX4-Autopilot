#!/usr/bin/env python3

# Copyright (c) 2018-2019, Ulf Magnusson
# SPDX-License-Identifier: ISC

"""
Writes a configuration file where as many symbols as possible are set to 'y'.

The default output filename is '.config'. A different filename can be passed
in the KCONFIG_CONFIG environment variable.

Usage for the Linux kernel:

  $ make [ARCH=<arch>] scriptconfig SCRIPT=Kconfiglib/allyesconfig.py
"""
import kconfiglib
import os


exception_list = [
    'DRIVERS_BOOTLOADERS', # Only used for bootloader target
    'MODULES_PX4IOFIRMWARE', # Only needed for PX4IO firmware itself, maybe fix through dependencies
    'BOARD_LTO', # Experimental
    'BOARD_TESTING', # Don't build test suite
    'BOARD_CONSTRAINED_FLASH', # only used to reduce flash size
    'BOARD_NO_HELP', # only used to reduce flash size
    'BOARD_CONSTRAINED_MEMORY', # only used to reduce flash size
    'BOARD_EXTERNAL_METADATA', # only used to reduce flash size
    'BOARD_CRYPTO', # Specialized use
    'BOARD_PROTECTED', # Experimental for MPU use
    'DRIVERS_LIGHTS_RGBLED_PWM', # Only on specific boards, needs dependency fixing
    'DRIVERS_LIGHTS_NEOPIXEL', # Only on specific boards, needs dependency fixing
    'DRIVERS_DISTANCE_SENSOR_LIGHTWARE_SF45_SERIAL', # Only on specific boards, needs dependency fixing
    'PARAM_PRIMARY', # Plainly broken
    'PARAM_REMOTE', # Plainly broken
    'DRIVERS_ACTUATORS_VOXL_ESC', # Dependency need fixing, requires VOXL_ESC_DEFAULT_XXX
    'DRIVERS_VOXL2_IO', # Dependency need fixing, requires VOXL2_IO_DEFAULT_XX
    'DRIVERS_BAROMETER_TCBP001TA', # Requires hardcoded PX4_SPI_BUS_BARO mapping
    'DRIVERS_DISTANCE_SENSOR_BROADCOM_AFBRS50', # Requires hardcoded PX4_SPI_BUS_BARO mapping
    'DRIVERS_DISTANCE_SENSOR_SRF05', # Requires hardcoded GPIO_ULTRASOUND
    'DRIVERS_PPS_CAPTURE', # Requires PPS GPIO config
    'DRIVERS_PWM_INPUT', # Requires PWM config
    'DRIVERS_RPM_CAPTURE', # Requires PPS GPIO config
    'DRIVERS_TEST_PPM', # PIN config not portable
    'DRIVERS_TATTU_CAN', # Broken needs fixing
    'MODULES_REPLAY', # Fails on NuttX targets maybe force POSIX dependency?
    'SYSTEMCMDS_HIST', # This module can only be used on boards that enable BOARD_ENABLE_LOG_HISTORY
    'SYSTEMCMDS_GPIO', # PIN config not portable
    'SYSTEMCMDS_SHUTDOWN', # Needs dependency checking
    'EXAMPLES_DYN_HELLO', # NuttX doesn't support dynamic linking
    'SYSTEMCMDS_DYN', # NuttX doesn't support dynamic linking
    'DRIVERS_RPI_RC_IN', # RPI specific driver
    'SYSTEMCMDS_I2C_LAUNCHER', #  undefined reference to `system',
    'MODULES_MUORB_APPS', # Weird QURT/Posix package doesn't work on x86 px4 sitl
    'MODULES_SIMULATION_SIMULATOR_SIH', # Causes compile errors
    'MODULES_SPACECRAFT', # Clashes with Control Allocation (mom's spaghetti code)
]

exception_list_sitl = [
    'DRIVERS_ADC', # Fails I2C dependencies
    'COMMON_BAROMETERS', # Fails I2C dependencies
    'DRIVERS_BAROMETER', # Fails I2C dependencies
    'DRIVERS_BATT_SMBUS', # Fails I2C dependencies in SMBus library
    'COMMON_DIFFERENTIAL_PRESSURE', # Fails I2C dependencies
    'DRIVERS_DIFFERENTIAL_PRESSURE', # Fails I2C dependencies
    'COMMON_DISTANCE_SENSOR', # Fails I2C dependencies
    'DRIVERS_DISTANCE_SENSOR', # Fails I2C dependencies
    'COMMON_HYGROMETERS', # Fails I2C dependencies
    'DRIVERS_HIWONDER_EMM', # Fails I2C dependencies
    'DRIVERS_HYGROMETER', # Fails I2C dependencies
    'COMMON_IMU', # Fails I2C dependencies
    'DRIVERS_IMU', # Fails I2C dependencies
    'DRIVERS_IRLOCK', # Fails I2C dependencies
    'COMMON_LIGHT', # Fails I2C dependencies
    'DRIVERS_LIGHTS', # Fails I2C dependencies
    'DRIVERS_MAGNETOMETER', # Fails I2C dependencies
    'COMMON_MAGNETOMETER', # Fails I2C dependencies
    'DRIVERS_OPTICAL_FLOW', # Fails I2C dependencies
    'COMMON_OPTICAL_FLOW', # Fails I2C dependencies
    'COMMON_OSD', # Fails I2C dependencies
    'DRIVERS_OSD', # Fails I2C dependencies
    'DRIVERS_PCA9685', # Fails I2C dependencies
    'DRIVERS_POWER_MONITOR', # Fails I2C dependencies
    'DRIVERS_RPM', # Fails I2C dependencies
    'COMMON_RPM', # Fails I2C dependencies
    'DRIVERS_SMART_BATTERY', # Fails I2C dependencies in SMBus library
    'DRIVERS_TELEMETRY', # Fails I2C dependencies
    'COMMON_TELEMETRY', # Fails I2C dependencies
    'DRIVERS_TEMPERATURE', # Fails I2C dependencies
    'COMMON_UWB', # Fail in UWB_PORT_CFG
    'DRIVERS_UWB', # Fail in UWB_PORT_CFG
    'DRIVERS_HEATER', # GPIO config failure
    'DRIVERS_AUTERION_AUTOSTARTER', # Sitl doesn't provide a builtin lib
    'DRIVERS_CYPHAL', # Sitl doesn't provide a memalign function
    'DRIVERS_GPIO', # Sitl doesn't provide a mcp-common lib
    'DRIVERS_ADC_BOARD_ADC', # Fails HW dependencies, I think this only works on NuttX
    'DRIVERS_CAMERA_CAPTURE', # GPIO config failure
    'DRIVERS_SAFETY_BUTTON', # GPIO config failure
    'DRIVERS_TAP_ESC', # No nuttx/arch.h
    'DRIVERS_DSHOT', # No Posix driver, I think this only works on NuttX
    'DRIVERS_PWM_OUT', # No Posix driver, I think this only works on NuttX
    'SYSTEMCMDS_REBOOT', # Sitl can't reboot
    'MODULES_BATTERY_STATUS', # Sitl doesn't provide a power brick
    'SYSTEMCMDS_SERIAL_PASSTHRU', # Not supported in SITL
    'SYSTEMCMDS_SERIAL_TEST', # Not supported in SITL
    'SYSTEMCMDS_SD_STRESS', # Not supported in SITL
    'SYSTEMCMDS_SD_BENCH', # Not supported in SITL
    'SYSTEMCMDS_I2CDETECT', # Not supported in SITL
    'SYSTEMCMDS_DMESG', # Not supported in SITL
    'SYSTEMCMDS_USB_CONNECTED', # Not supported in SITL
    'SYSTEMCMDS_MFT_CFG', # Not supported in SITL
    'MODULES_SPACECRAFT', # Clashes with Control Allocation (mom's spaghetti code)
]

def main():
    kconf = kconfiglib.standard_kconfig(__doc__)


    if 'BASE_DEFCONFIG' in os.environ:
        kconf.load_config(os.environ['BASE_DEFCONFIG'])

    if 'MODEL' in os.environ:
        if os.environ['MODEL'] == 'sitl':
            for sym in kconf.unique_defined_syms:
                if sym.name.startswith(tuple(exception_list_sitl)):
                    exception_list.append(sym.name)


    # See allnoconfig.py
    kconf.warn = False

    # Try to set all symbols to 'y'. Dependencies might truncate the value down
    # later, but this will at least give the highest possible value.
    #
    # Assigning 0/1/2 to non-bool/tristate symbols has no effect (int/hex
    # symbols still take a string, because they preserve formatting).
    for sym in kconf.unique_defined_syms:
        # Set choice symbols to 'm'. This value will be ignored for choices in
        # 'y' mode (the "normal" mode), which will instead just get their
        # default selection, but will set all symbols in m-mode choices to 'm',
        # which is as high as they can go.
        #
        # Here's a convoluted example of how you might get an m-mode choice
        # even during allyesconfig:
        #
        #   choice
        #           tristate "weird choice"
        #           depends on m
        if sym.name not in exception_list:
            sym.set_value(1 if sym.choice else 2)

    # Set all choices to the highest possible mode
    for choice in kconf.unique_choices:
        choice.set_value(2)

    kconf.warn = True

    kconf.load_allconfig("allyes.config")

    print(kconf.write_config())


if __name__ == "__main__":
    main()
