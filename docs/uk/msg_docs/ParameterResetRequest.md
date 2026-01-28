# ParameterResetRequest (повідомлення UORB)

ParameterResetRequest : Параметр скинути запит. Використовується в основному для скидання одного або всіх параметрів значення на віддаленому

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterResetRequest.msg)

```c
# ParameterResetRequest : Used by the primary to reset one or all parameter value(s) on the remote

uint64 timestamp
uint16 parameter_index

bool reset_all              # If this is true then ignore parameter_index

uint8 ORB_QUEUE_LENGTH = 4

```
