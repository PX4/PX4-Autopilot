
float32 roll	        # body angular rates in NED frame
float32 pitch	        # body angular rates in NED frame
float32 yaw			    # body angular rates in NED frame
float32 thrust	        # thrust normalized to 0..1

# For 3D thrust control
float32[3] thrust_3d    # 3D thrust in body frame, the norm of thrust_3d should be equal to thrust
bool thrust_3d_valid    # Set to true if 3D thrust is valid

# TOPICS vehicle_rates_setpoint mc_virtual_rates_setpoint fw_virtual_rates_setpoint
