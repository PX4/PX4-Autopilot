---
pageClass: is-wide-page
---

# LedControl (повідомлення UORB)

Керування світлодіодами: керування одним чи кількома світлодіодами. These are the externally visible LED's, not the board LED's.

**TOPICS:** led_control

## Fields

| Назва                           | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                                                                                    |
| ------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                       | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                               |
| led_mask   | `uint8`  |                                                                  |            | bitmask which LED(s) to control, set to 0xff for all                                                                                 |
| color                           | `uint8`  |                                                                  |            | see COLOR\_\*                                                                                                                     |
| mode                            | `uint8`  |                                                                  |            | see MODE\_\*                                                                                                                      |
| num_blinks | `uint8`  |                                                                  |            | how many times to blink (number of on-off cycles if mode is one of MODE_BLINK_\*) . Set to 0 for infinite            |
| priority                        | `uint8`  |                                                                  |            | priority: higher priority events will override current lower priority events (see MAX_PRIORITY) |

## Constants

| Назва                                                                                         | Тип     | Значення | Опис                                                                                                                                             |
| --------------------------------------------------------------------------------------------- | ------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a href="#COLOR_OFF"></a> COLOR_OFF                                      | `uint8` | 0        | this is only used in the drivers                                                                                                                 |
| <a href="#COLOR_RED"></a> COLOR_RED                                      | `uint8` | 1        |                                                                                                                                                  |
| <a href="#COLOR_GREEN"></a> COLOR_GREEN                                  | `uint8` | 2        |                                                                                                                                                  |
| <a href="#COLOR_BLUE"></a> COLOR_BLUE                                    | `uint8` | 3        |                                                                                                                                                  |
| <a href="#COLOR_YELLOW"></a> COLOR_YELLOW                                | `uint8` | 4        |                                                                                                                                                  |
| <a href="#COLOR_PURPLE"></a> COLOR_PURPLE                                | `uint8` | 5        |                                                                                                                                                  |
| <a href="#COLOR_AMBER"></a> COLOR_AMBER                                  | `uint8` | 6        |                                                                                                                                                  |
| <a href="#COLOR_CYAN"></a> COLOR_CYAN                                    | `uint8` | 7        |                                                                                                                                                  |
| <a href="#COLOR_WHITE"></a> COLOR_WHITE                                  | `uint8` | 8        |                                                                                                                                                  |
| <a href="#MODE_OFF"></a> MODE_OFF                                        | `uint8` | 0        | turn LED off                                                                                                                                     |
| <a href="#MODE_ON"></a> MODE_ON                                          | `uint8` | 1        | turn LED on                                                                                                                                      |
| <a href="#MODE_DISABLED"></a> MODE_DISABLED                              | `uint8` | 2        | disable this priority (switch to lower priority setting)                                                                      |
| <a href="#MODE_BLINK_SLOW"></a> MODE_BLINK_SLOW     | `uint8` | 3        |                                                                                                                                                  |
| <a href="#MODE_BLINK_NORMAL"></a> MODE_BLINK_NORMAL | `uint8` | 4        |                                                                                                                                                  |
| <a href="#MODE_BLINK_FAST"></a> MODE_BLINK_FAST     | `uint8` | 5        |                                                                                                                                                  |
| <a href="#MODE_BREATHE"></a> MODE_BREATHE                                | `uint8` | 6        | continuously increase & decrease brightness (solid color if driver does not support it)                   |
| <a href="#MODE_FLASH"></a> MODE_FLASH                                    | `uint8` | 7        | two fast blinks (on/off) with timing as in MODE_BLINK_FAST and then off for a while |
| <a href="#MAX_PRIORITY"></a> MAX_PRIORITY                                | `uint8` | 2        | maximum priority (minimum is 0)                                                                                               |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH   | `uint8` | 8        | needs to match BOARD_MAX_LEDS                                                                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LedControl.msg)

:::details
Click here to see original file

```c
# LED control: control a single or multiple LED's.
# These are the externally visible LED's, not the board LED's

uint64 timestamp		# time since system start (microseconds)

# colors
uint8 COLOR_OFF = 0 # this is only used in the drivers
uint8 COLOR_RED = 1
uint8 COLOR_GREEN = 2
uint8 COLOR_BLUE = 3
uint8 COLOR_YELLOW = 4
uint8 COLOR_PURPLE = 5
uint8 COLOR_AMBER = 6
uint8 COLOR_CYAN = 7
uint8 COLOR_WHITE = 8

# LED modes definitions
uint8 MODE_OFF = 0 # turn LED off
uint8 MODE_ON = 1  # turn LED on
uint8 MODE_DISABLED = 2  # disable this priority (switch to lower priority setting)
uint8 MODE_BLINK_SLOW = 3
uint8 MODE_BLINK_NORMAL = 4
uint8 MODE_BLINK_FAST = 5
uint8 MODE_BREATHE = 6 # continuously increase & decrease brightness (solid color if driver does not support it)
uint8 MODE_FLASH = 7 # two fast blinks (on/off) with timing as in MODE_BLINK_FAST and then off for a while

uint8 MAX_PRIORITY = 2 # maximum priority (minimum is 0)


uint8 led_mask # bitmask which LED(s) to control, set to 0xff for all
uint8 color # see COLOR_*
uint8 mode # see MODE_*
uint8 num_blinks # how many times to blink (number of on-off cycles if mode is one of MODE_BLINK_*) . Set to 0 for infinite
                 # in MODE_FLASH it is the number of cycles. Max number of blinks: 122 and max number of flash cycles: 20
uint8 priority # priority: higher priority events will override current lower priority events (see MAX_PRIORITY)

uint8 ORB_QUEUE_LENGTH = 8      # needs to match BOARD_MAX_LEDS
```

:::
