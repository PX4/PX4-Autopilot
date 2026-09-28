---
pageClass: is-wide-page
---

# SensorGnssSpectrum (UORB message)

GNSS spectrum analysis.

Reports a basic RF spectrum analysis for a GNSS receiver, decoded from the u-blox UBX-MON-SPAN
message (enabled via the GPS_UBX_SPECTRUM parameter).
Published by the `gps` driver (UBX protocol only), once per RF block, via the block-specific topics below.
The center frequency of bin i is: f(i) = center_frequency + spectrum_span \* (i - 127) / 256

**TOPICS:** sensor_gnss_spectrum_block0 sensor_gnss_spectrum_block1 sensor_gnss_spectrum_block2

## Fields

| Name                                                                    | Type         | Unit [Frame] | Range/Enum | Description                                                                                  |
| ----------------------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                     | `uint64`     | us           |            | Time since system start                                                                      |
| <a id="fld_timestamp_sample"></a>timestamp_sample                       | `uint64`     | us           |            | Timestamp of the raw data                                                                    |
| <a id="fld_device_id"></a>device_id                                     | `uint32`     |              |            | Unique device ID for the sensor that does not change between power cycles                    |
| <a id="fld_block_id"></a>block_id                                       | `uint8`      |              |            | Only used for multi-topic publishing. Use center_frequency to determine which RF band it is. |
| <a id="fld_spectrum"></a>spectrum                                       | `uint8[256]` | (2^-2)dB     |            | Spectrum magnitude per bin (number of points = spectrum_span / resolution)                   |
| <a id="fld_spectrum_span"></a>spectrum_span                             | `uint32`     | Hz           |            | Frequency span covered by the spectrum                                                       |
| <a id="fld_resolution"></a>resolution                                   | `uint32`     | Hz           |            | Frequency resolution of the spectrum (bin width)                                             |
| <a id="fld_center_frequency"></a>center_frequency                       | `uint32`     | Hz           |            | Center frequency of the spectrum span                                                        |
| <a id="fld_programmable_gain_amplifier"></a>programmable_gain_amplifier | `uint8`      | dB           |            | Programmable gain amplifier setting                                                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGnssSpectrum.msg)

::: details Click here to see original file

```c
# GNSS spectrum analysis
#
# Reports a basic RF spectrum analysis for a GNSS receiver, decoded from the u-blox UBX-MON-SPAN
# message (enabled via the GPS_UBX_SPECTRUM parameter).
# Published by the `gps` driver (UBX protocol only), once per RF block, via the block-specific topics below.
# The center frequency of bin i is: f(i) = center_frequency + spectrum_span * (i - 127) / 256

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint32 device_id # Unique device ID for the sensor that does not change between power cycles

uint8 block_id # [-] Only used for multi-topic publishing. Use center_frequency to determine which RF band it is.

uint8[256] spectrum # [(2^-2)dB] Spectrum magnitude per bin (number of points = spectrum_span / resolution)
uint32 spectrum_span # [Hz] Frequency span covered by the spectrum
uint32 resolution # [Hz] Frequency resolution of the spectrum (bin width)
uint32 center_frequency # [Hz] Center frequency of the spectrum span
uint8 programmable_gain_amplifier # [dB] Programmable gain amplifier setting

# One topic per RF block ID
# TOPICS sensor_gnss_spectrum_block0 sensor_gnss_spectrum_block1 sensor_gnss_spectrum_block2
```

:::
