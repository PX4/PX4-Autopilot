# Gripper (повідомлення UORB)

# Використовується для виклику активації в захоплювачі, яка відображена на конкретний вихід в модулі розподілу керування

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Gripper.msg)

```c
## Used to command an actuation in the gripper, which is mapped to a specific output in the control allocation module

uint64 timestamp

int8 command		# Commanded state for the gripper
int8 COMMAND_GRAB = 0
int8 COMMAND_RELEASE = 1

```
