# Relative position of precision land target in body frame

uint64 timestamp		# time since system start (microseconds)

bool rel_pos_valid		# Flag showing whether relative position is valid

float32 x_rel_body			# X/Front position of target, relative to vehicle (body frame) [meters]
float32 y_rel_body			# Y/Right position of target, relative to vehicle (body frame) [meters]
float32 z_rel_body			# Z/Down position of target, relative to vehicle (body frame) [meters]

float32 var_x_rel_body		# X/Front position variance [meters^2]
float32 var_y_rel_body		# Y/Right position variance [meters^2]
float32 var_z_rel_body		# Z/Down position variance [meters^2]

float32[4] q            # Quaternion rotation from the sensor frame to the NED earth frame
