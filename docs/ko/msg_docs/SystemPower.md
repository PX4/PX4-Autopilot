---
pageClass: is-wide-page
---

# SystemPower (UORB message)

**TOPICS:** system_power

## Fields

| 명칭                                                                                | 형식           | Unit [Frame] | Range/Enum | 설명                                                                               |
| --------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------- |
| timestamp                                                                         | `uint64`     |                                                                  |            | time since system start (microseconds)                        |
| voltage5v_v                                                  | `float32`    |                                                                  |            | peripheral 5V rail voltage                                                       |
| voltage_payload_v                       | `float32`    |                                                                  |            | payload rail voltage                                                             |
| sensors3v3                                                                        | `float32[4]` |                                                                  |            | Sensors 3V3 rail voltage                                                         |
| sensors3v3_valid                                             | `uint8`      |                                                                  |            | Sensors 3V3 rail voltage was read (bitfield). |
| usb_connected                                                | `uint8`      |                                                                  |            | USB is connected when 1                                                          |
| brick_valid                                                  | `uint8`      |                                                                  |            | brick bits power is good when bit 1                                              |
| usb_valid                                                    | `uint8`      |                                                                  |            | USB is valid when 1                                                              |
| servo_valid                                                  | `uint8`      |                                                                  |            | servo power is good when 1                                                       |
| periph_5v_oc                            | `uint8`      |                                                                  |            | peripheral overcurrent when 1                                                    |
| hipower_5v_oc                           | `uint8`      |                                                                  |            | high power peripheral overcurrent when 1                                         |
| comp_5v_valid                           | `uint8`      |                                                                  |            | 5V to companion valid                                                            |
| can1_gps1_5v_valid | `uint8`      |                                                                  |            | 5V for CAN1/GPS1 valid                                                           |
| payload_v_valid                         | `uint8`      |                                                                  |            | payload rail voltage is valid                                                    |

## Constants

| 명칭                                                                                                | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#BRICK1_VALID_SHIFTS"></a> BRICK1_VALID_SHIFTS | `uint8` | 0     |    |
| <a href="#BRICK1_VALID_MASK"></a> BRICK1_VALID_MASK     | `uint8` | 1     |    |
| <a href="#BRICK2_VALID_SHIFTS"></a> BRICK2_VALID_SHIFTS | `uint8` | 1     |    |
| <a href="#BRICK2_VALID_MASK"></a> BRICK2_VALID_MASK     | `uint8` | 2     |    |
| <a href="#BRICK3_VALID_SHIFTS"></a> BRICK3_VALID_SHIFTS | `uint8` | 2     |    |
| <a href="#BRICK3_VALID_MASK"></a> BRICK3_VALID_MASK     | `uint8` | 4     |    |
| <a href="#BRICK4_VALID_SHIFTS"></a> BRICK4_VALID_SHIFTS | `uint8` | 3     |    |
| <a href="#BRICK4_VALID_MASK"></a> BRICK4_VALID_MASK     | `uint8` | 8     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SystemPower.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
float32 voltage5v_v		# peripheral 5V rail voltage
float32 voltage_payload_v	# payload rail voltage
float32[4] sensors3v3		# Sensors 3V3 rail voltage
uint8 sensors3v3_valid		# Sensors 3V3 rail voltage was read (bitfield).
uint8 usb_connected		# USB is connected when 1
uint8 brick_valid		# brick bits power is good when bit 1
uint8 usb_valid 		# USB is valid when 1
uint8 servo_valid		# servo power is good when 1
uint8 periph_5v_oc		# peripheral overcurrent when 1
uint8 hipower_5v_oc		# high power peripheral overcurrent when 1
uint8 comp_5v_valid		# 5V to companion valid
uint8 can1_gps1_5v_valid	# 5V for CAN1/GPS1 valid
uint8 payload_v_valid		# payload rail voltage is valid

uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8
```

:::
