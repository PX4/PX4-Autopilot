uint64 timestamp                # time since system start (microseconds)

float32[3] fe          # estimated external force - FRD body frame [Nm]
float32[3] me          # estimated external moment - FRD body frame [Nm]

float32[2] x_thrust         # RLS estimated thrust constant (k_f*1e6) and reduction bottom rotors (k_r*1e6)
float32[3] x_offset         # RLS estimated CoM offset - z axis is not reliable for quad/octo [m]

float32[3] fi          # estimated actuator force - FRD body frame [Nm]
float32[3] mi          # estimated actuator moment - FRD body frame [Nm]

bool interaction_flag  # interaction flag used for estimation
bool valid
