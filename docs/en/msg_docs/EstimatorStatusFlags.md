# EstimatorStatusFlags (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStatusFlags.msg)

```c
uint64 timestamp                          # time since system start (microseconds)
uint64 timestamp_sample                   # the timestamp of the raw data (microseconds)


# filter control status
uint32 control_status_changes # number of filter control status (cs) changes
bool cs_tilt_align            #  0 - true if the filter tilt alignment is complete
bool cs_yaw_align             #  1 - true if the filter yaw alignment is complete
bool cs_gnss_pos              #  2 - true if GNSS position measurement fusion is intended
bool cs_opt_flow              #  3 - true if optical flow measurements fusion is intended
bool cs_mag_hdg               #  4 - true if a simple magnetic yaw heading fusion is intended
bool cs_mag_3d                #  5 - true if 3-axis magnetometer measurement fusion is intended
bool cs_mag_dec               #  6 - true if synthetic magnetic declination measurements fusion is intended
bool cs_in_air                #  7 - true when the vehicle is airborne
bool cs_wind                  #  8 - true when wind velocity is being estimated
bool cs_baro_hgt              #  9 - true when baro data is being fused
bool cs_rng_hgt               # 10 - true when range finder data is being fused for height aiding
bool cs_gps_hgt               # 11 - true when GPS altitude is being fused
bool cs_ev_pos                # 12 - true when local position data fusion from external vision is intended
bool cs_ev_yaw                # 13 - true when yaw data from external vision measurements fusion is intended
bool cs_ev_hgt                # 14 - true when height data from external vision measurements is being fused
bool cs_fuse_beta             # 15 - true when synthetic sideslip measurements are being fused
bool cs_mag_field_disturbed   # 16 - true when the mag field does not match the expected strength
bool cs_fixed_wing            # 17 - true when the vehicle is operating as a fixed wing vehicle
bool cs_mag_fault             # 18 - true when the magnetometer has been declared faulty and is no longer being used
bool cs_fuse_aspd             # 19 - true when airspeed measurements are being fused
bool cs_gnd_effect            # 20 - true when protection from ground effect induced static pressure rise is active
bool cs_rng_stuck             # 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough
bool cs_gnss_yaw              # 22 - true when yaw (not ground course) data fusion from a GPS receiver is intended
bool cs_mag_aligned_in_flight # 23 - true when the in-flight mag field alignment has been completed
bool cs_ev_vel                # 24 - true when local frame velocity data fusion from external vision measurements is intended
bool cs_synthetic_mag_z       # 25 - true when we are using a synthesized measurement for the magnetometer Z component
bool cs_vehicle_at_rest       # 26 - true when the vehicle is at rest
bool cs_gnss_yaw_fault        # 27 - true when the GNSS heading has been declared faulty and is no longer being used
bool cs_rng_fault             # 28 - true when the range finder has been declared faulty and is no longer being used
bool cs_inertial_dead_reckoning # 29 - true if we are no longer fusing measurements that constrain horizontal velocity drift
bool cs_wind_dead_reckoning     # 30 - true if we are navigationg reliant on wind relative measurements
bool cs_rng_kin_consistent      # 31 - true when the range finder kinematic consistency check is passing
bool cs_fake_pos                # 32 - true when fake position measurements are being fused
bool cs_fake_hgt                # 33 - true when fake height measurements are being fused
bool cs_gravity_vector          # 34 - true when gravity vector measurements are being fused
bool cs_mag                     # 35 - true if 3-axis magnetometer measurement fusion (mag states only) is intended
bool cs_ev_yaw_fault            # 36 - true when the EV heading has been declared faulty and is no longer being used
bool cs_mag_heading_consistent  # 37 - true when the heading obtained from mag data is declared consistent with the filter
bool cs_aux_gpos                # 38 - true if auxiliary global position measurement fusion is intended
bool cs_rng_terrain             # 39 - true if we are fusing range finder data for terrain
bool cs_opt_flow_terrain        # 40 - true if we are fusing flow data for terrain
bool cs_valid_fake_pos          # 41 - true if a valid constant position is being fused
bool cs_constant_pos            # 42 - true if the vehicle is at a constant position
bool cs_baro_fault	        # 43 - true when the current baro has been declared faulty and is no longer being used
bool cs_gnss_vel                # 44 - true if GNSS velocity measurement fusion is intended

# fault status
uint32 fault_status_changes   # number of filter fault status (fs) changes
bool fs_bad_mag_x             #  0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
bool fs_bad_mag_y             #  1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
bool fs_bad_mag_z             #  2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
bool fs_bad_hdg               #  3 - true if the fusion of the heading angle has encountered a numerical error
bool fs_bad_mag_decl          #  4 - true if the fusion of the magnetic declination has encountered a numerical error
bool fs_bad_airspeed          #  5 - true if fusion of the airspeed has encountered a numerical error
bool fs_bad_sideslip          #  6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
bool fs_bad_optflow_x         #  7 - true if fusion of the optical flow X axis has encountered a numerical error
bool fs_bad_optflow_y         #  8 - true if fusion of the optical flow Y axis has encountered a numerical error
bool fs_bad_acc_vertical      # 10 - true if bad vertical accelerometer data has been detected
bool fs_bad_acc_clipping      # 11 - true if delta velocity data contains clipping (asymmetric railing)


# innovation test failures
uint32 innovation_fault_status_changes    # number of innovation fault status (reject) changes
bool reject_hor_vel                       #  0 - true if horizontal velocity observations have been rejected
bool reject_ver_vel                       #  1 - true if vertical velocity observations have been rejected
bool reject_hor_pos                       #  2 - true if horizontal position observations have been rejected
bool reject_ver_pos                       #  3 - true if vertical position observations have been rejected
bool reject_yaw                           #  7 - true if the yaw observation has been rejected
bool reject_airspeed                      #  8 - true if the airspeed observation has been rejected
bool reject_sideslip                      #  9 - true if the synthetic sideslip observation has been rejected
bool reject_hagl                          # 10 - true if the height above ground observation has been rejected
bool reject_optflow_x                     # 11 - true if the X optical flow observation has been rejected
bool reject_optflow_y                     # 12 - true if the Y optical flow observation has been rejected

```
