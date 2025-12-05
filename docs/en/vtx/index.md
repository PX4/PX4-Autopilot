# Analog Video Transmitters

Analog Video Transmitters (VTX) can be controlled by PX4 via a half-duplex UART connection implementing the SmartAudio v1, v2, and v2.1 and Tramp protocols.

The protocols allow writing and reading:
- device status.
- transmission frequency in MHz or via band and channel index.
- transmission power in dBm or mW.
- operation modes.

These settings are controlled directly by parameters:
- `VTX_BAND`: Sets the VTX band.
- `VTX_CHANNEL`: Sets the VTX channel.
- `VTX_POWER`: Sets the VTX power level.
- `VTX_FREQUENCY`: Sets the VTX frequency in MHz (overrides band and channel if set).
- `VTX_PIT_MODE`: Sets the VTX to pit mode (reduced power).

In addition, the driver implements configuration storage for:
- a frequency table for VTX channels and bands.
- a power table for different power levels.
- mapping of AUX channels to VTX settings for manual control.

This configuration depends on the specific connected VTX device and can be configured using the command line interface in airframes.
Using the configuration, the VTX settings parameters can be controlled via AUX channels or via CRSF MSP commands.


## Getting Started

Connect the SmartAudio or Tramp pin of the VTX to a free serial port on the flight controller.
Then set the following parameters:
- `VTX_SER_CFG`: Select the serial port used for VTX communication.
- `VTX_PROTOCOL`: Selects the VTX protocol (SmartAudio or Tramp).

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

Note that without a power level table, the power level to power mapping is unknown and will default to 0 mW. As shown in this example, setting power to 0 can trigger the pit mode on the VTX, even if pit mode is disabled in the parameters.


## Power Levels

The VTX table contains the power level and power mappings.
You can configure the power levels using the following commands:

```
# Set table name ≤15 characters
vtx name "Peak THOR T67"

# Set the power values that are sent to the VTX for each power level index
# Note: SmartAudio v1 and v2 use index values!
vtx powervalues 0 1 2 3 4
# Note: SmartAudio v2.1 uses dBm values instead!
#       vtx powervalues 14 1 2 3 4
# Note: Tramp uses mW values instead!
#       vtx powervalues 25 200 500 1000 3000

# Set the corresponding power in mW for each power level index
# These are used for status reporting.
vtx powerlabels 25 200 500 1000 3000

# Set number of power levels
vtx powerlevels 5

# Save configuration
vtx save
```

This will create a VTX table with 5 power levels.
```nsh> vtx status
INFO  [vtx] VTX table "Peak THOR T67":
INFO  [vtx] Power levels:
INFO  [vtx]   1:  0 =   25 mW
INFO  [vtx]   2:  1 =  200 mW
INFO  [vtx]   3:  2 =  500 mW
INFO  [vtx]   4:  3 = 1000 mW
INFO  [vtx]   5:  4 = 3000 mW
```

## Frequency Table

The VTX table contains the frequency and power level mappings for different bands and channels.
You can configure the VTX table using the following commands:

```
# Set the name of each band and the frequencies of each channel
# vtx band <band index> <band name max 15 chars> <band character> <max 8x frequencies>
vtx band 1  BAND_A  A  6110 6130 6150 6170 6190 6210 6230 6250
vtx band 2  BAND_B  b  6270 6290 6310 6330 6350 6370 6390 6410
vtx band 3  BAND_E  E  6430 6450 6470 6490 6510 6530 6550 6570
vtx band 4  BAND_F  F  6590 6610 6630 6650 6670 6690 6710 6730
vtx band 5  BAND_R  r  6750 6770 6790 6810 6830 6850 6870 6890
vtx band 6  BAND_P  P  6910 6930 6950 6970 6990 7010 7030 7050
vtx band 7  BAND_H  H  7070 7090 7110 7130 7150 7170 7190 7210
vtx band 8  BAND_U  U  6115 6265 6425 6585 6745 6905 7065 7185

# Set number of bands
vtx bands 8

# Save configuration
vtx save
```

This will create a VTX table with 8 bands.
```
nsh> vtx status
INFO  [vtx] VTX table "Peak THOR T67":
INFO  [vtx]   A: BAND_A          = 6110 6130 6150 6170 6190 6210 6230 6250
INFO  [vtx]   b: BAND_B          = 6270 6290 6310 6330 6350 6370 6390 6410
INFO  [vtx]   E: BAND_E          = 6430 6450 6470 6490 6510 6530 6550 6570
INFO  [vtx]   F: BAND_F          = 6590 6610 6630 6650 6670 6690 6710 6730
INFO  [vtx]   r: BAND_R          = 6750 6770 6790 6810 6830 6850 6870 6890
INFO  [vtx]   P: BAND_P          = 6910 6930 6950 6970 6990 7010 7030 7050
INFO  [vtx]   H: BAND_H          = 7070 7090 7110 7130 7150 7170 7190 7210
INFO  [vtx]   U: BAND_U          = 6115 6265 6425 6585 6745 6905 7065 7185
```

## AUX Map

```
vtx map  0 7 7 1 0 900 1025
vtx map  1 7 7 2 0 1025 1100
vtx map  2 7 7 4 0 1100 1175
vtx map  3 7 7 6 0 1175 1225
vtx map  4 7 7 8 0 1225 1300
vtx map  5 7 7 1 0 1300 1375
vtx map  6 7 6 3 0 1375 1425
vtx map  7 7 6 5 0 1425 1500
vtx map  8 7 6 6 0 1500 1575
vtx map  9 7 6 7 0 1575 1625
vtx map 10 7 3 4 0 1625 1700
vtx map 11 7 5 2 0 1700 1775
vtx map 12 7 5 3 0 1775 1825
vtx map 13 7 5 7 0 1825 1900
vtx map 14 7 5 8 0 1900 1975
vtx map 15 7 3 8 0 1975 2100
vtx map 16 3 0 0 -1 900 1250
vtx map 17 3 0 0 1 1250 1525
vtx map 18 3 0 0 2 1525 1650
vtx map 19 3 0 0 3 1650 1875
vtx map 20 3 0 0 4 1875 2010
```


## CRSF MSP



