# GNSS redundancy status published by vehicle_gps_position and consumed by commander

uint64 timestamp                 # time since system start (microseconds)

uint8 num_receivers_online       # receivers publishing fresh data
uint8 num_receivers_fixed        # receivers with fix_type >= 3
uint8 num_receivers_required     # configured minimum from SYS_HAS_NUM_GNSS (0 = no minimum)
uint8 peak_receivers_fixed       # highest fixed count seen since boot

bool below_required              # num_receivers_fixed < num_receivers_required
bool dropped_below_peak          # num_receivers_fixed < peak_receivers_fixed
bool divergence_detected         # two receivers diverge beyond the gate after sustain

float32 divergence_m             # last-measured horizontal distance between the two receivers
float32 divergence_gate_m        # current gate (lever-arm separation + sigma * RMS(eph))
