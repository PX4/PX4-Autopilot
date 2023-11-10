uint64 timestamp			# time since system start (microseconds)

float32[4] wheel_angle #Wheel angle of encoder, if two wheels wheel_angle[0] = right wheel and wheel_angle[1] = left wheel. Else [0][1] = right and [2][3] are left.
float32[4] wheel_speed #Wheel speed of encoder, if two wheels wheel_angle[0] = right wheel and wheel_angle[1] = left wheel. Else [0][1] = right and [2][3] are left.

