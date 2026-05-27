---
pageClass: is-wide-page
---

# LandingTargetInnovations (UORB message)

**TOPICS:** landing_target_innovations

## Fields

| Name                                    | Type      | Unit [Frame] | Range/Enum | Description                            |
| --------------------------------------- | --------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp     | `uint64`  |              |            | time since system start (microseconds) |
| <a id="fld_innov_x"></a>innov_x         | `float32` |              |            |
| <a id="fld_innov_y"></a>innov_y         | `float32` |              |            |
| <a id="fld_innov_cov_x"></a>innov_cov_x | `float32` |              |            |
| <a id="fld_innov_cov_y"></a>innov_cov_y | `float32` |              |            |

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
