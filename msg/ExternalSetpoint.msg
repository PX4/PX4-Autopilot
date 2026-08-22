# External setpoint contribution
#
# Velocity/acceleration setpoint streamed from an external source, typically a
# companion computer. It is fused into Position and Mission mode by the
# flight_mode_manager as a *temporary deviation*: while fresh setpoints arrive the
# vehicle follows them, and when the stream stops the underlying Position/Mission
# setpoint resumes automatically.
#
# The mechanism is application-agnostic. Obstacle avoidance is the motivating use
# case, but nothing here is specific to it - any onboard behaviour that wants to
# contribute a bounded correction without taking over the vehicle can use it.
#
# NED world frame. Setting an axis to NaN means "do not command this axis".

uint64 timestamp        # time since system start (microseconds)

float32[3] velocity     # NED velocity setpoint [m/s], NaN per-axis if not commanded
float32[3] acceleration # NED acceleration setpoint [m/s^2], NaN per-axis if not commanded

float32 yaw             # absolute yaw setpoint [rad], NaN if not commanded
float32 yawspeed        # yaw rate setpoint [rad/s], NaN if not commanded

bool valid              # true if this setpoint should be considered for fusion
