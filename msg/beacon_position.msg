# Relative and absolute position of precision land beacon in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

bool rel_pos_valid		# Flag showing whether relative position is valid
bool rel_vel_valid		# Flag showing whether relative velocity is valid
float32 x_rel			# X/north position of beacon, relative to vehicle (navigation frame) [meters]
float32 y_rel			# Y/east position of beacon, relative to vehicle (navigation frame) [meters]
float32 vx_rel			# X/north velocity  of beacon, relative to vehicle (navigation frame) [meters/second]
float32 vy_rel			# Y/east velocity of beacon, relative to vehicle (navigation frame) [meters/second]

float32 x_unfilt		# DEBUG X/north unfiltered relative position of beacon, [meters]
float32 y_unfilt		# DEBUG Y/east unfiltered relative position of beacon, [meters]

# XXX the position and especially velocity in local frame should probably be removed
bool local_pos_valid	# Flag showing whether local position is valid
bool local_vel_valid	# Flag showing whether local velocity is valid
float32 x_local			# X/north position of beacon, in local coordinates (inertial frame) [meters]
float32 y_local			# Y/east position of beacon, in local coordinates (inertial frame) [meters]
float32 vx_local		# X/north velocity of beacon, in local coordinates (inertial frame) [meters/second] XXX REMOVE?
float32 vy_local		# Y/east velocity of beacon, in local coordinates (linertial frame [meters/second] XXX REMOVE?

float32 cov_x_rel		# X/north position variance [meters^2]
float32 cov_y_rel		# Y/east position variance [meters^2]
float32 cov_vx_rel		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_rel		# Y/east velocity variance [(meters/second)^2]
