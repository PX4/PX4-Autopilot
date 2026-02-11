# Analog Video Transmitters

Analog Video Transmitters (VTX) can be controlled by PX4 via a half-duplex UART connection implementing the SmartAudio v1, v2, and v2.1 and Tramp protocols.

The protocols allow writing and reading:

- device status.
- transmission frequency in MHz or via band and channel index.
- transmission power in dBm or mW.
- operation modes.

VTX settings are controlled by parameters and optionally via RC AUX channels or CRSF MSP commands.
The driver stores frequency and power tables that map band/channel indices to actual transmission values.
Configuration is device-specific and set up using the command line interface.

## Початок роботи

Connect the SmartAudio or Tramp pin of the VTX to the TX pin of a free serial port on the flight controller.
Then set the following parameters:

- `VTX_SER_CFG`: Select the serial port used for VTX communication.
- `VTX_DEVICE`: Selects the VTX device (generic SmartAudio/Tramp or a specific device).

Note that since the VTX communication is half-duplex, you can, for example, use the single-pin Radio Controller port for the VTX and use a full duplex TELEM port for CRSF communication.

You should now be able to see the VTX device in the driver status:

```
nsh> vtx status
INFO  [vtx] UART device: /dev/ttyS4
INFO  [vtx] VTX table "UNINITIALIZED":
INFO  [vtx] Power levels:
INFO  [vtx] RC mapping: Disabled
INFO  [vtx] Parameters:
INFO  [vtx]   band: 1
INFO  [vtx]   channel: 1
INFO  [vtx]   frequency: 0 MHz
INFO  [vtx]   power level: 1
INFO  [vtx]   power: 0 = 0 mW
INFO  [vtx]   pit mode: off
INFO  [vtx] SmartAudio v2:
INFO  [vtx]   band: 1
INFO  [vtx]   channel: 1
INFO  [vtx]   frequency: 6110 MHz
INFO  [vtx]   power level: 1
INFO  [vtx]   power: 0 mW
INFO  [vtx]   pit mode: on
INFO  [vtx]   lock: unlocked
```

:::warning
Without a configured power table, power mappings are unknown and default to 0 mW.
Some VTX devices enter pit mode when power is set to 0, regardless of the `VTX_PIT_MODE` parameter.
:::

## VTX Table Configuration

The VTX table stores frequency and power mappings for your specific device.

The manufacturer usually provides this information in the form of a JSON file that can be translated into a [Betaflight CLI command set](https://www.betaflight.com/docs/development/VTX#vtx-table) that this driver implements for compatibility.

### Power Level Configuration

```
# Set table name ≤16 characters
vtxtable name "Peak THOR T67"

# Set the power values that are sent to the VTX for each power level index
# Note: SmartAudio v1 and v2 use index values!
vtxtable powervalues 0 1 2 3 4
# Note: SmartAudio v2.1 uses dBm values instead!
#       vtxtable powervalues 14 23 27 30 35
# Note: Tramp uses mW values instead!
#       vtxtable powervalues 25 200 500 1000 3000

# Set the corresponding power labels for each power level index ≤4 characters.
# These are used for status reporting.
vtxtable powerlabels 25 200 500 1W 3W

# Set number of power levels
vtxtable powerlevels 5

# Save configuration
vtxtable save
```

This will create a VTX table with 5 power levels.

```nsh> vtxtable status
INFO  [vtxtable] VTX table "Peak THOR T67":
INFO  [vtxtable] Power levels:
INFO  [vtxtable]   1:  0 = 25
INFO  [vtxtable]   2:  1 = 200
INFO  [vtxtable]   3:  2 = 500
INFO  [vtxtable]   4:  3 = 1W
INFO  [vtxtable]   5:  4 = 3W
```

### Frequency Table Configuration

```
# Set the name of each band and the frequencies of each channel
vtxtable band 1 BAND_A A FACTORY 6110 6130 6150 6170 6190 6210 6230 6250
vtxtable band 2 BAND_B B FACTORY 6270 6290 6310 6330 6350 6370 6390 6410
vtxtable band 3 BAND_E E FACTORY 6430 6450 6470 6490 6510 6530 6550 6570
vtxtable band 4 BAND_F F FACTORY 6590 6610 6630 6650 6670 6690 6710 6730
vtxtable band 5 BAND_R R FACTORY 6750 6770 6790 6810 6830 6850 6870 6890
vtxtable band 6 BAND_P P FACTORY 6910 6930 6950 6970 6990 7010 7030 7050
vtxtable band 7 BAND_H H FACTORY 7070 7090 7110 7130 7150 7170 7190 7210
vtxtable band 8 BAND_U U FACTORY 6115 6265 6425 6585 6745 6905 7065 7185

# Set number of bands and channels
vtxtable bands 8
vtxtable channels 8

# Save configuration
vtxtable save
```

This will create a VTX table with 8 bands and 8 channels.
Note that FACTORY sends the band and channel indexes to the VTX device and they use their internal frequency mapping. In this mode the frequency is just for indication purposes.
In contrast, CUSTOM would send the actual frequency values to the VTX device, but not all devices support this mode.
Setting a frequency to zero will skip setting it.

```
nsh> vtxtable status
INFO  [vtxtable] VTX table 8x8: Peak THOR T67
INFO  [vtxtable]  A: BAND_A          = 6110 6130 6150 6170 6190 6210 6230 6250
INFO  [vtxtable]  B: BAND_B          = 6270 6290 6310 6330 6350 6370 6390 6410
INFO  [vtxtable]  E: BAND_E          = 6430 6450 6470 6490 6510 6530 6550 6570
INFO  [vtxtable]  F: BAND_F          = 6590 6610 6630 6650 6670 6690 6710 6730
INFO  [vtxtable]  R: BAND_R          = 6750 6770 6790 6810 6830 6850 6870 6890
INFO  [vtxtable]  P: BAND_P          = 6910 6930 6950 6970 6990 7010 7030 7050
INFO  [vtxtable]  H: BAND_H          = 7070 7090 7110 7130 7150 7170 7190 7210
INFO  [vtxtable]  U: BAND_U          = 6115 6265 6425 6585 6745 6905 7065 7185
```

### Table Constraints

Maximum table dimensions:

- ≤24 bands each with ≤16 channels and ≤32GHz frequency values.
- ≤16 power levels.
- ≤16 characters table name.
- ≤12 characters band name and 1 character band letter.
- ≤4 characters power label length (to support "2.5W").

## AUX Channel Mapping

The AUX mapping feature allows you to control VTX settings using RC AUX channels.
Each mapping entry defines an AUX channel range that triggers a specific VTX configuration.

To enable AUX mapping, set `VTX_MAP_CONFIG` to one of the following values:

- `0`: Disabled
- `1`: Disabled (reserved for CRSF MSP integration)
- `2`: Map AUX channels to power level control only
- `3`: Map AUX channels to band and channel control only
- `4`: Map AUX channels to all settings (power, band, and channel)

### Configuring AUX Map Entries

Use the following command format to add mapping entries:

```
vtxtable <index> <aux_channel> <band> <channel> <power> <start_pwm> <end_pwm>
```

Параметри:

- `index`: Map entry index (0-159)
- `aux_channel`: AUX channel number (3-19, where AUX1=3)
- `band`: Target band (1-24, or 0 to leave unchanged)
- `channel`: Target channel (1-16, or 0 to leave unchanged)
- `power`: Power level (1-16, 0 to leave unchanged, or -1 for pit mode)
- `start_pwm`: Start of PWM range in microseconds (typically 900-2100)
- `end_pwm`: End of PWM range in microseconds (typically 900-2100)

:::info
AUX channel numbering starts from 3 (AUX1=channel 3) to account for the first four RC channels 0-3 used for flight control.
:::

Example configuration for a 6-position dial controlling band/channel on AUX4 (channel 7):

```
vtx  0 7 7 1 0  900 1025
vtx  1 7 7 2 0 1025 1100
vtx  2 7 7 4 0 1100 1175
vtx  3 7 7 6 0 1175 1225
vtx  4 7 7 8 0 1225 1300
vtx  5 7 3 8 0 1300 2100
```

Example configuration for power control on AUX3 (channel 6):

```
vtxtable 16 6 0 0 -1  900 1250
vtxtable 17 6 0 0  1 1250 1525
vtxtable 18 6 0 0  2 1525 1650
vtxtable 19 6 0 0  3 1650 1875
vtxtable 20 6 0 0  4 1875 2010
```

Save the configuration with:

```
vtxtable save
```

The map status can be verified with `vtxtable status`.

## CRSF MSP Integration

When using a CRSF receiver with MSP support, you can control VTX settings directly from your transmitter using MSP commands sent over the CRSF link.
This feature must be enabled at compile time with the `VTX_CRSF_MSP_SUPPORT` Kconfig option.

To enable CRSF MSP control, set `VTX_MAP_CONFIG` to one of:

- `1`: MSP controls both frequency (band/channel) and power
- `2`: MSP controls frequency (band/channel) only, AUX controls power
- `3`: MSP controls power only, AUX controls band/channel

When MSP integration is active, the driver responds to `MSP_SET_VTX_CONFIG` (0x59) commands.
The transmitter can send band, channel, frequency, power level, and pit mode settings via MSP, which are automatically mapped to the corresponding PX4 parameters.

:::tip
The MSP integration allows seamless VTX control from transmitters that support VTX configuration via Lua scripts or built-in VTX menus without requiring additional hardware switches.
:::

## Build Configuration

Both the VTX driver and VTX table support are configured via Kconfig options.

Key configuration options:

- `VTX_CRSF_MSP_SUPPORT`: Enables CRSF MSP command support (default: disabled)
- `VTXTABLE_CONFIG_FILE`: File path for persistent configuration (default: `/fs/microsd/vtx_config`)
- `VTXTABLE_AUX_MAP`: Enables AUX channel mapping (default: disabled)

## Parameter Reference

### VTX Settings Parameters

- `VTX_BAND` (0-23): Frequency band selection (Band 1-24 in UI)
- `VTX_CHANNEL` (0-15): Channel within band (Channel 1-16 in UI)
- `VTX_FREQUENCY` (0-32000): Direct frequency in MHz (overrides band/channel when non-zero)
- `VTX_POWER` (0-15): Power level (Level 1-16 in UI, as configured in table)
- `VTX_PIT_MODE` (boolean): Pit mode for reduced power (default: disabled)

### Налаштування параметрів

- `VTX_SER_CFG`: Serial port assignment for VTX communication
- `VTX_MAP_CONFIG`: Controls how VTX settings are mapped:
  - Without `VTX_CRSF_MSP_SUPPORT`:
    - `0`: Disabled
    - `1`: Disabled
    - `2`: AUX controls power only
    - `3`: AUX controls band/channel only
    - `4`: AUX controls both power and band/channel
  - With `VTX_CRSF_MSP_SUPPORT`:
    - `0`: Disabled
    - `1`: MSP controls both frequency and power
    - `2`: MSP controls frequency, AUX controls power
    - `3`: MSP controls power, AUX controls band/channel
    - `4`: Not used with MSP support
- `VTX_DEVICE`: Device-specific configuration (see below)

## Device-Specific Configuration

The `VTX_DEVICE` parameter allows device-specific workarounds.
It encodes both the protocol type and device variant:

- Low byte (bits 0-7): Protocol selection
  - `0`: SmartAudio (default)
  - `1`: Tramp
- High byte (bits 8-15): Device-specific variant
  - `0`: Generic device
  - `20`: Peak THOR T67
  - `40`: Rush Max Solo

### Known Device Workarounds

**Peak THOR T67** (`VTX_DEVICE` = 5120):
This device incorrectly reports pit mode status but otherwise functions normally.
The driver applies a workaround to override the reported status with the actual configured state.

For generic devices, use `VTX_DEVICE` = 0 (SmartAudio) or `VTX_DEVICE` = 1 (Tramp).
