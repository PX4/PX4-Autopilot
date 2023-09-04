# Target estimator Kalman Filter state

uint64 timestamp		# time since system start (microseconds)

bool rel_pos_valid		# Flag showing whether relative position is valid
bool rel_vel_valid		# Flag showing whether relative velocity is valid

float32 x_rel			# X/north position of target, relative to vehicle (navigation frame) [meters]
float32 y_rel			# Y/east position of target, relative to vehicle (navigation frame) [meters]
float32 z_rel			# Z/down position of target, relative to vehicle (navigation frame) [meters]

float32 vx_rel			# X/north velocity of target, relative to vehicle (navigation frame) [meters/second]
float32 vy_rel			# Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]
float32 vz_rel			# Z/down velocity of target, relative to vehicle (navigation frame) [meters/second]

float32 vx_target		# X/north absolute velocity  of target (navigation frame) [meters/second]
float32 vy_target		# Y/east absolute velocity of target (navigation frame) [meters/second]
float32 vz_target		# Z/down absolute velocity of target (navigation frame) [meters/second]

float32 x_bias			# X/north bias of drone and target GPS units (navigation frame) [meters]
float32 y_bias			# Y/east bias of drone and target GPS units (navigation frame) [meters]
float32 z_bias			# Z/down bias of drone and target GPS units (navigation frame) [meters]

float32 ax_target		# X/north absolute acceleration  of target (navigation frame) [meters/second^2]
float32 ay_target		# Y/east absolute acceleration of target (navigation frame) [meters/second^2]
float32 az_target		# Z/down absolute acceleration of target (navigation frame) [meters/second^2]

float32 cov_x_rel		# X/north position variance [meters^2]
float32 cov_y_rel		# Y/east position variance [meters^2]
float32 cov_z_rel		# Z/down position variance [meters^2]

float32 cov_vx_rel		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_rel		# Y/east velocity variance [(meters/second)^2]
float32 cov_vz_rel		# Z/down velocity variance [(meters/second)^2]

float32 cov_x_bias		# X/north bias variance [(meters)^2]
float32 cov_y_bias		# Y/east bias variance [(meters)^2]
float32 cov_z_bias		# Z/down bias variance [(meters)^2]

float32 cov_vx_target		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_target		# Y/east velocity variance [(meters/second)^2]
float32 cov_vz_target		# Z/down velocity variance [(meters/second)^2]

float32 cov_ax_target		# X/north velocity variance [(meters/second^2)^2]
float32 cov_ay_target		# Y/east velocity variance [(meters/second^2)^2]
float32 cov_az_target		# Z/down velocity variance [(meters/second^2)^2]
