uint64 timestamp                          # time since system start (microseconds)


bool ev_data_arrived   #when have ev data that arrived from the dds
uint8 reset_counter   # number of accepted EV resets this session (matches vehicle_odometry.reset_counter)
bool heading_alligned_with_ev   # when the heading is aligned with the EV (used to count resets)
bool pos_alligned_with_ev   # when the position is aligned with the EV (used to count resets)
bool multy_sensor_z_velocity_error   # when the ev z velocity and the baro deriv are not similar (indicating a possible error in the ev velocity)
bool quality_init_check_fail   # when the quality average during the settle window is above the threshold, indicating a possible error in the EV data (e.g. bad EV fusion configuration, or EV not really moving which makes the quality metric less meaningful)
bool vel_3d_init_check_fail   # when the 3D velocity average during the settle window is above the threshold, indicating a possible error in the EV data (e.g. bad EV fusion configuration, or EV not really moving which makes the quality metric less meaningful)
bool error_find
uint8 status
float32 baro_data

