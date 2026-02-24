---
pageClass: is-wide-page
---

# GpioConfig (UORB message)

GPIO configuration.

**TOPICS:** gpio_config

## Fields

| Name      | Type     | Unit [Frame] | Range/Enum | Description                            |
| --------- | -------- | ------------ | ---------- | -------------------------------------- |
| timestamp | `uint64` |              |            | time since system start (microseconds) |
| device_id | `uint32` |              |            | Device id                              |
| mask      | `uint32` |              |            | Pin mask                               |
| state     | `uint32` |              |            | Initial pin output state               |
| config    | `uint32` |              |            |

## Constants

| Name                                                          | Type     | Value | Description |
| ------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#INPUT			"></a> INPUT                                     | `uint32` | 0     | 0x0000      |
| <a id="#OUTPUT			"></a> OUTPUT                                   | `uint32` | 1     | 0x0001      |
| <a id="#PULLUP			"></a> PULLUP                                   | `uint32` | 16    | 0x0010      |
| <a id="#PULLDOWN			"></a> PULLDOWN                               | `uint32` | 32    | 0x0020      |
| <a id="#OPENDRAIN		"></a> OPENDRAIN                             | `uint32` | 256   | 0x0100      |
| <a id="#INPUT_FLOATING		"></a> INPUT_FLOATING                   | `uint32` | 0     | 0x0000      |
| <a id="#INPUT_PULLUP		"></a> INPUT_PULLUP                       | `uint32` | 16    | 0x0010      |
| <a id="#INPUT_PULLDOWN		"></a> INPUT_PULLDOWN                   | `uint32` | 32    | 0x0020      |
| <a id="#OUTPUT_PUSHPULL		"></a> OUTPUT_PUSHPULL                 | `uint32` | 0     | 0x0000      |
| <a id="#OUTPUT_OPENDRAIN		"></a> OUTPUT_OPENDRAIN               | `uint32` | 256   | 0x0100      |
| <a id="#OUTPUT_OPENDRAIN_PULLUP	"></a> OUTPUT_OPENDRAIN_PULLUP | `uint32` | 272   | 0x0110      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioConfig.msg)

::: details Click here to see original file

```c
# GPIO configuration

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 mask				# Pin mask
uint32 state				# Initial pin output state

# Configuration Mask
# Bit 0-3: Direction: 0=Input, 1=Output
# Bit 4-7: Input Config: 0=Floating, 1=PullUp, 2=PullDown
# Bit 8-12: Output Config: 0=PushPull, 1=OpenDrain
# Bit 13-31: Reserved
uint32 INPUT			= 0	# 0x0000
uint32 OUTPUT			= 1	# 0x0001
uint32 PULLUP			= 16	# 0x0010
uint32 PULLDOWN			= 32	# 0x0020
uint32 OPENDRAIN		= 256	# 0x0100

uint32 INPUT_FLOATING		= 0	# 0x0000
uint32 INPUT_PULLUP		= 16	# 0x0010
uint32 INPUT_PULLDOWN		= 32	# 0x0020

uint32 OUTPUT_PUSHPULL		= 0	# 0x0000
uint32 OUTPUT_OPENDRAIN		= 256	# 0x0100
uint32 OUTPUT_OPENDRAIN_PULLUP	= 272	# 0x0110

uint32 config
```

:::
