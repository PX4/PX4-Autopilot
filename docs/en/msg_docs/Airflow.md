---
pageClass: is-wide-page
---

# Airflow (UORB message)

Estimated airflow around the vehicle.

This is published by the wind_estimator_3d module.

**TOPICS:** airflow airflow_groundtruth

## Fields

| Name                                             | Type      | Unit [Frame] | Range/Enum | Description                                                   |
| ------------------------------------------------ | --------- | ------------ | ---------- | -------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp              | `uint64`  | us           |            | Time since system start                                       |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  | us           |            | Timestamp of the raw data                                      |
| <a id="fld_u"></a>u                              | `float32` | m/s          |            | Body-relative air relative velocity component / X direction   |
| <a id="fld_v"></a>v                              | `float32` | m/s          |            | Body-relative air relative velocity component / Y direction   |
| <a id="fld_w"></a>w                              | `float32` | m/s          |            | Body-relative air relative velocity component / Z direction   |
| <a id="fld_windspeed_north"></a>windspeed_north  | `float32` | m/s          |            | Wind component in north / X direction                          |
| <a id="fld_windspeed_east"></a>windspeed_east    | `float32` | m/s          |            | Wind component in east / Y direction                            |
| <a id="fld_windspeed_down"></a>windspeed_down    | `float32` | m/s          |            | Wind component in down / Z direction                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Airflow.msg)

::: details Click here to see original file

```c
# Estimated airflow around the vehicle
#
# This is published by the wind_estimator_3d module.

uint64 timestamp                 # [us] Time since system start
uint64 timestamp_sample          # [us] Timestamp of the raw data
float32 u   			 # [m/s] Body-relative air relative velocity component / X direction
float32 v        		 # [m/s] Body-relative air relative velocity component / Y direction
float32 w               	 # [m/s] Body-relative air relative velocity component / Z direction
float32 windspeed_north	# [m/s] Wind component in north / X direction
float32 windspeed_east # [m/s] Wind component in east / Y direction
float32 windspeed_down # [m/s] Wind component in down / Z direction

# TOPICS airflow airflow_groundtruth
```

:::
