# Small additive torque used by multicopter autotune, after controller output filters.
# Expired commands and commands from a different flight mode must be ignored.
uint64 timestamp # [us] Publication time
float32[3] torque # [-] Normalized additive torque
uint8 nav_state # Flight mode in which the experiment started
