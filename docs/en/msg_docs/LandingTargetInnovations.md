---
pageClass: is-wide-page
---

# LandingTargetInnovations (UORB message)

**TOPICS:** landing_target_innovations

## Fields

| Name        | Type      | Unit [Frame] | Range/Enum | Description                            |
| ----------- | --------- | ------------ | ---------- | -------------------------------------- |
| timestamp   | `uint64`  |              |            | time since system start (microseconds) |
| innov_x     | `float32` |              |            |
| innov_y     | `float32` |              |            |
| innov_cov_x | `float32` |              |            |
| innov_cov_y | `float32` |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingTargetInnovations.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
# Innovation of landing target position estimator
float32 innov_x
float32 innov_y

# Innovation covariance of landing target position estimator
float32 innov_cov_x
float32 innov_cov_y
```

:::
