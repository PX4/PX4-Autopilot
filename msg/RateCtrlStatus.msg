uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ

# Integrator bounds for body x,y,z axis. Equal to +-the integrator limit parameter unless the
# control allocator reported unallocated torque, which lowers the bound on the refused side.
float32[3] integrator_limit_upper
float32[3] integrator_limit_lower
