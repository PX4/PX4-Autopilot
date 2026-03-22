---
pageClass: is-wide-page
---

# NeuralControl (UORB message)

Neural control.

Debugging topic for the Neural controller, logs the inputs and output vectors of the neural network, and the time it takes to run
Publisher: mc_nn_control
Subscriber: logger

**TOPICS:** neural_control

## Fields

| 参数名                                  | 类型            | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                    |
| ------------------------------------ | ------------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                            | `uint64`      | us                                                               |            | Time since system start                                                                                                                                               |
| observation                          | `float32[15]` |                                                                  |            | Observation vector (pos error (3), att (6d), lin vel (3), ang vel (3)) |
| network_output  | `float32[4]`  |                                                                  |            | Output from neural network                                                                                                                                            |
| controller_time | `int32`       | us                                                               |            | Time spent from input to output                                                                                                                                       |
| inference_time  | `int32`       | us                                                               |            | Time spent for NN inference                                                                                                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NeuralControl.msg)

:::details
Click here to see original file

```c
# Neural control
#
# Debugging topic for the Neural controller, logs the inputs and output vectors of the neural network, and the time it takes to run
# Publisher: mc_nn_control
# Subscriber: logger

uint64 timestamp # [us] Time since system start

float32[15] observation # Observation vector (pos error (3), att (6d), lin vel (3), ang vel (3))
float32[4] network_output # Output from neural network

int32 controller_time # [us] Time spent from input to output
int32 inference_time # [us] Time spent for NN inference
```

:::
