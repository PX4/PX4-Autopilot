# GNSS altitude drift offset detected by EKF2.
#
# Published when EKF2 detects that the fused GNSS altitude has drifted relative to both the
# barometer and the integrated GNSS vertical velocity. Consumers (e.g. HomePosition) apply
# this offset to keep AMSL references aligned with the drift-corrected altitude.

uint64 timestamp # [us] time since system start
float32 altitude_offset # [m] cumulative altitude offset since detector start; consumers apply the delta to be resilient to missed messages
