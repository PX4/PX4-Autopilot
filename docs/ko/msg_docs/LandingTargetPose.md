---
pageClass: is-wide-page
---

# LandingTargetPose (UORB message)

Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames.

**TOPICS:** landing_targetpose

## Fields

| 명칭                                                      | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                                                        |
| ------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                               | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                 |
| is_static                          | `bool`    |                                                                  |            | Flag indicating whether the landing target is static or moving with respect to the ground                                                                 |
| rel_pos_valid | `bool`    |                                                                  |            | Flag showing whether relative position is valid                                                                                                           |
| rel_vel_valid | `bool`    |                                                                  |            | Flag showing whether relative velocity is valid                                                                                                           |
| x_rel                              | `float32` |                                                                  |            | X/north position of target, relative to vehicle (navigation frame) [meters]        |
| y_rel                              | `float32` |                                                                  |            | Y/east position of target, relative to vehicle (navigation frame) [meters]         |
| z_rel                              | `float32` |                                                                  |            | Z/down position of target, relative to vehicle (navigation frame) [meters]         |
| vx_rel                             | `float32` |                                                                  |            | X/north velocity of target, relative to vehicle (navigation frame) [meters/second] |
| vy_rel                             | `float32` |                                                                  |            | Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]  |
| cov_x_rel     | `float32` |                                                                  |            | X/north position variance [meters^2]                                                                  |
| cov_y_rel     | `float32` |                                                                  |            | Y/east position variance [meters^2]                                                                   |
| cov_vx_rel    | `float32` |                                                                  |            | X/north velocity variance [(meters/second)^2]                                      |
| cov_vy_rel    | `float32` |                                                                  |            | Y/east velocity variance [(meters/second)^2]                                       |
| abs_pos_valid | `bool`    |                                                                  |            | Flag showing whether absolute position is valid                                                                                                           |
| x_abs                              | `float32` |                                                                  |            | X/north position of target, relative to origin (navigation frame) [meters]         |
| y_abs                              | `float32` |                                                                  |            | Y/east position of target, relative to origin (navigation frame) [meters]          |
| z_abs                              | `float32` |                                                                  |            | Z/down position of target, relative to origin (navigation frame) [meters]          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingTargetPose.msg)

:::details
Click here to see original file

```c
# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		# time since system start (microseconds)

bool is_static			# Flag indicating whether the landing target is static or moving with respect to the ground

bool rel_pos_valid		# Flag showing whether relative position is valid
bool rel_vel_valid		# Flag showing whether relative velocity is valid

float32 x_rel			# X/north position of target, relative to vehicle (navigation frame) [meters]
float32 y_rel			# Y/east position of target, relative to vehicle (navigation frame) [meters]
float32 z_rel			# Z/down position of target, relative to vehicle (navigation frame) [meters]

float32 vx_rel			# X/north velocity  of target, relative to vehicle (navigation frame) [meters/second]
float32 vy_rel			# Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]

float32 cov_x_rel		# X/north position variance [meters^2]
float32 cov_y_rel		# Y/east position variance [meters^2]

float32 cov_vx_rel		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_rel		# Y/east velocity variance [(meters/second)^2]

bool abs_pos_valid		# Flag showing whether absolute position is valid
float32 x_abs			# X/north position of target, relative to origin (navigation frame) [meters]
float32 y_abs			# Y/east position of target, relative to origin (navigation frame) [meters]
float32 z_abs			# Z/down position of target, relative to origin (navigation frame) [meters]
```

:::
