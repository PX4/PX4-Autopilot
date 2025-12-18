# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		# [us] time since system start

bool is_static			# [-] Flag indicating whether the landing target is static or moving with respect to the ground

bool rel_pos_valid		# [-] Flag showing whether relative position is valid
bool rel_vel_valid		# [-] Flag showing whether relative velocity is valid

float32 x_rel			# [m] X/north position of target, relative to vehicle (navigation frame)
float32 y_rel			# [m] Y/east position of target, relative to vehicle (navigation frame)
float32 z_rel			# [m] Z/down position of target, relative to vehicle (navigation frame)

float32 vx_rel			# [m/s] X/north velocity  of target, relative to vehicle (navigation frame)
float32 vy_rel			# [m/s] Y/east velocity of target, relative to vehicle (navigation frame)
float32 vz_rel			# [m/s] Z/down velocity of target, relative to vehicle (navigation frame)

float32 cov_x_rel		# [m^2] X/north position variance
float32 cov_y_rel		# [m^2] Y/east position variance
float32 cov_z_rel		# [m^2] Z/down position variance

float32 cov_vx_rel		# [(m/s)^2] X/north velocity variance
float32 cov_vy_rel		# [(m/s)^2] Y/east velocity variance]
float32 cov_vz_rel		# [(m/s)^2] Z/down velocity variance

bool abs_pos_valid		# [-] Flag showing whether absolute position is valid

float32 x_abs			# [m] X/north position of target, relative to origin (navigation frame)
float32 y_abs			# [m] Y/east position of target, relative to origin (navigation frame)
float32 z_abs			# [m] Z/down position of target, relative to origin (navigation frame)
