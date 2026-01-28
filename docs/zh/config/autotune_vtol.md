# Autotuning (VTOL)

Auto-tuning automates the process of tuning the PX4 rate and attitude PID controllers, which are the most important controllers for stable and responsive flight (other tuning is more "optional").

Tuning only needs to be done once, and is recommended unless you're using a vehicle that has already been tuned by the manufacturer (and not modified since).

Hybrid VTOL fixed-wing vehicles ("VTOL") must be tuned following the multicopter instructions in MC mode and then the fixed-wing instructions in FW mode:

- [Auto-tune (Multicopter)](../config/autotune_mc.md)
- [Auto-tune (Fixed-wing)](../config/autotune_fw.md)
