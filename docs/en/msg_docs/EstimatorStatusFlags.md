---
pageClass: is-wide-page
---

# EstimatorStatusFlags (UORB message)

**TOPICS:** estimator_status_flags

## Fields

| Name                                                                  | Type     | Unit [Frame] | Range/Enum | Description                                                                                      |
| --------------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                   | `uint64` |              |            | time since system start (microseconds)                                                           |
| <a id="fld_timestamp_sample"></a>timestamp_sample                     | `uint64` |              |            | the timestamp of the raw data (microseconds)                                                     |
| <a id="fld_control_status_changes"></a>control_status_changes         | `uint32` |              |            | number of filter control status (cs) changes                                                     |
| <a id="fld_cs_tilt_align"></a>cs_tilt_align                           | `bool`   |              |            | 0 - true if the filter tilt alignment is complete                                                |
| <a id="fld_cs_yaw_align"></a>cs_yaw_align                             | `bool`   |              |            | 1 - true if the filter yaw alignment is complete                                                 |
| <a id="fld_cs_gnss_pos"></a>cs_gnss_pos                               | `bool`   |              |            | 2 - true if GNSS position measurement fusion is intended                                         |
| <a id="fld_cs_opt_flow"></a>cs_opt_flow                               | `bool`   |              |            | 3 - true if optical flow measurements fusion is intended                                         |
| <a id="fld_cs_mag_hdg"></a>cs_mag_hdg                                 | `bool`   |              |            | 4 - true if a simple magnetic yaw heading fusion is intended                                     |
| <a id="fld_cs_mag_3d"></a>cs_mag_3d                                   | `bool`   |              |            | 5 - true if 3-axis magnetometer measurement fusion is intended                                   |
| <a id="fld_cs_mag_dec"></a>cs_mag_dec                                 | `bool`   |              |            | 6 - true if synthetic magnetic declination measurements fusion is intended                       |
| <a id="fld_cs_in_air"></a>cs_in_air                                   | `bool`   |              |            | 7 - true when the vehicle is airborne                                                            |
| <a id="fld_cs_wind"></a>cs_wind                                       | `bool`   |              |            | 8 - true when wind velocity is being estimated                                                   |
| <a id="fld_cs_baro_hgt"></a>cs_baro_hgt                               | `bool`   |              |            | 9 - true when baro data is being fused                                                           |
| <a id="fld_cs_rng_hgt"></a>cs_rng_hgt                                 | `bool`   |              |            | 10 - true when range finder data is being fused for height aiding                                |
| <a id="fld_cs_gps_hgt"></a>cs_gps_hgt                                 | `bool`   |              |            | 11 - true when GPS altitude is being fused                                                       |
| <a id="fld_cs_ev_pos"></a>cs_ev_pos                                   | `bool`   |              |            | 12 - true when local position data fusion from external vision is intended                       |
| <a id="fld_cs_ev_yaw"></a>cs_ev_yaw                                   | `bool`   |              |            | 13 - true when yaw data from external vision measurements fusion is intended                     |
| <a id="fld_cs_ev_hgt"></a>cs_ev_hgt                                   | `bool`   |              |            | 14 - true when height data from external vision measurements is being fused                      |
| <a id="fld_cs_fuse_beta"></a>cs_fuse_beta                             | `bool`   |              |            | 15 - true when synthetic sideslip measurements are being fused                                   |
| <a id="fld_cs_mag_field_disturbed"></a>cs_mag_field_disturbed         | `bool`   |              |            | 16 - true when the mag field does not match the expected strength                                |
| <a id="fld_cs_fixed_wing"></a>cs_fixed_wing                           | `bool`   |              |            | 17 - true when the vehicle is operating as a fixed wing vehicle                                  |
| <a id="fld_cs_mag_fault"></a>cs_mag_fault                             | `bool`   |              |            | 18 - true when the magnetometer has been declared faulty and is no longer being used             |
| <a id="fld_cs_fuse_aspd"></a>cs_fuse_aspd                             | `bool`   |              |            | 19 - true when airspeed measurements are being fused                                             |
| <a id="fld_cs_gnd_effect"></a>cs_gnd_effect                           | `bool`   |              |            | 20 - true when protection from ground effect induced static pressure rise is active              |
| <a id="fld_cs_rng_stuck"></a>cs_rng_stuck                             | `bool`   |              |            | 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough |
| <a id="fld_cs_gnss_yaw"></a>cs_gnss_yaw                               | `bool`   |              |            | 22 - true when yaw (not ground course) data fusion from a GPS receiver is intended               |
| <a id="fld_cs_mag_aligned_in_flight"></a>cs_mag_aligned_in_flight     | `bool`   |              |            | 23 - true when the in-flight mag field alignment has been completed                              |
| <a id="fld_cs_ev_vel"></a>cs_ev_vel                                   | `bool`   |              |            | 24 - true when local frame velocity data fusion from external vision measurements is intended    |
| <a id="fld_cs_synthetic_mag_z"></a>cs_synthetic_mag_z                 | `bool`   |              |            | 25 - true when we are using a synthesized measurement for the magnetometer Z component           |
| <a id="fld_cs_vehicle_at_rest"></a>cs_vehicle_at_rest                 | `bool`   |              |            | 26 - true when the vehicle is at rest                                                            |
| <a id="fld_cs_gnss_yaw_fault"></a>cs_gnss_yaw_fault                   | `bool`   |              |            | 27 - true when the GNSS heading has been declared faulty and is no longer being used             |
| <a id="fld_cs_rng_fault"></a>cs_rng_fault                             | `bool`   |              |            | 28 - true when the range finder has been declared faulty and is no longer being used             |
| <a id="fld_cs_inertial_dead_reckoning"></a>cs_inertial_dead_reckoning | `bool`   |              |            | 29 - true if we are no longer fusing measurements that constrain horizontal velocity drift       |
| <a id="fld_cs_wind_dead_reckoning"></a>cs_wind_dead_reckoning         | `bool`   |              |            | 30 - true if we are navigationg reliant on wind relative measurements                            |
| <a id="fld_cs_rng_kin_consistent"></a>cs_rng_kin_consistent           | `bool`   |              |            | 31 - true when the range finder kinematic consistency check is passing                           |
| <a id="fld_cs_fake_pos"></a>cs_fake_pos                               | `bool`   |              |            | 32 - true when fake position measurements are being fused                                        |
| <a id="fld_cs_fake_hgt"></a>cs_fake_hgt                               | `bool`   |              |            | 33 - true when fake height measurements are being fused                                          |
| <a id="fld_cs_gravity_vector"></a>cs_gravity_vector                   | `bool`   |              |            | 34 - true when gravity vector measurements are being fused                                       |
| <a id="fld_cs_mag"></a>cs_mag                                         | `bool`   |              |            | 35 - true if 3-axis magnetometer measurement fusion (mag states only) is intended                |
| <a id="fld_cs_ev_yaw_fault"></a>cs_ev_yaw_fault                       | `bool`   |              |            | 36 - true when the EV heading has been declared faulty and is no longer being used               |
| <a id="fld_cs_mag_heading_consistent"></a>cs_mag_heading_consistent   | `bool`   |              |            | 37 - true when the heading obtained from mag data is declared consistent with the filter         |
| <a id="fld_cs_aux_gpos"></a>cs_aux_gpos                               | `bool`   |              |            | 38 - true if auxiliary global position measurement fusion is intended                            |
| <a id="fld_cs_rng_terrain"></a>cs_rng_terrain                         | `bool`   |              |            | 39 - true if we are fusing range finder data for terrain                                         |
| <a id="fld_cs_opt_flow_terrain"></a>cs_opt_flow_terrain               | `bool`   |              |            | 40 - true if we are fusing flow data for terrain                                                 |
| <a id="fld_cs_valid_fake_pos"></a>cs_valid_fake_pos                   | `bool`   |              |            | 41 - true if a valid constant position is being fused                                            |
| <a id="fld_cs_constant_pos"></a>cs_constant_pos                       | `bool`   |              |            | 42 - true if the vehicle is at a constant position                                               |
| <a id="fld_cs_baro_fault"></a>cs_baro_fault                           | `bool`   |              |            | 43 - true when the current baro has been declared faulty and is no longer being used             |
| <a id="fld_cs_gnss_vel"></a>cs_gnss_vel                               | `bool`   |              |            | 44 - true if GNSS velocity measurement fusion is intended                                        |
| <a id="fld_cs_gnss_fault"></a>cs_gnss_fault                           | `bool`   |              |            | 45 - true if GNSS true if GNSS measurements (lat, lon, vel) have been declared faulty            |
| <a id="fld_cs_yaw_manual"></a>cs_yaw_manual                           | `bool`   |              |            | 46 - true if yaw has been set manually                                                           |
| <a id="fld_cs_gnss_hgt_fault"></a>cs_gnss_hgt_fault                   | `bool`   |              |            | 47 - true if GNSS true if GNSS measurements (alt) have been declared faulty                      |
| <a id="fld_cs_in_transition"></a>cs_in_transition                     | `bool`   |              |            | 48 - true if the vehicle is in vtol transition                                                   |
| <a id="fld_cs_heading_observable"></a>cs_heading_observable           | `bool`   |              |            | 49 - true when heading is observable                                                             |
| <a id="fld_fault_status_changes"></a>fault_status_changes             | `uint32` |              |            | number of filter fault status (fs) changes                                                       |
| <a id="fld_fs_bad_mag_x"></a>fs_bad_mag_x                             | `bool`   |              |            | 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error              |
| <a id="fld_fs_bad_mag_y"></a>fs_bad_mag_y                             | `bool`   |              |            | 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error              |
| <a id="fld_fs_bad_mag_z"></a>fs_bad_mag_z                             | `bool`   |              |            | 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error              |
| <a id="fld_fs_bad_hdg"></a>fs_bad_hdg                                 | `bool`   |              |            | 3 - true if the fusion of the heading angle has encountered a numerical error                    |
| <a id="fld_fs_bad_mag_decl"></a>fs_bad_mag_decl                       | `bool`   |              |            | 4 - true if the fusion of the magnetic declination has encountered a numerical error             |
| <a id="fld_fs_bad_airspeed"></a>fs_bad_airspeed                       | `bool`   |              |            | 5 - true if fusion of the airspeed has encountered a numerical error                             |
| <a id="fld_fs_bad_sideslip"></a>fs_bad_sideslip                       | `bool`   |              |            | 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error        |
| <a id="fld_fs_bad_optflow_x"></a>fs_bad_optflow_x                     | `bool`   |              |            | 7 - true if fusion of the optical flow X axis has encountered a numerical error                  |
| <a id="fld_fs_bad_optflow_y"></a>fs_bad_optflow_y                     | `bool`   |              |            | 8 - true if fusion of the optical flow Y axis has encountered a numerical error                  |
| <a id="fld_fs_bad_acc_vertical"></a>fs_bad_acc_vertical               | `bool`   |              |            | 10 - true if bad vertical accelerometer data has been detected                                   |
| <a id="fld_fs_bad_acc_clipping"></a>fs_bad_acc_clipping               | `bool`   |              |            | 11 - true if delta velocity data contains clipping (asymmetric railing)                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStatusFlags.msg)

::: details Click here to see original file

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
bool cs_gnss_fault              # 45 - true if GNSS true if GNSS measurements (lat, lon, vel) have been declared faulty
bool cs_yaw_manual              # 46 - true if yaw has been set manually
bool cs_gnss_hgt_fault          # 47 - true if GNSS true if GNSS measurements (alt) have been declared faulty
bool cs_in_transition           # 48 - true if the vehicle is in vtol transition
bool cs_heading_observable      # 49 - true when heading is observable

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
```

:::
