# Joint-space command for the six independently tilting Voliro rotors.
uint64 timestamp                         # [us] Time since system start
uint64 timestamp_sample                  # [us] Source controller sample timestamp
float32[6] angle                         # [rad] Radial-axis joint angles
uint8 valid_mask                         # One bit per valid joint command
