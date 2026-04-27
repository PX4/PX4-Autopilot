uint64 timestamp                          # time since system start (microseconds)



bool clutch_engaged          # the state of the clutch ok, to avoid publishing same state multiple times
bool vio_is_init              # the state of the vio is init, to avoid publishing same state multiple times
bool vins_process_started
bool msekf_client_start         # flag that the msekf client is start
bool msekf_service_ok           # the msekf ok and publish the data
bool quality_ok_2_clutch        # the quality is ok to clutch at the start of the session
bool velocity_ok_2_clutch       # the velocity is ok to clutch at the start of the session
bool over_limit_velo
bool last_vins_process_running
bool vio_process_reset
bool movment_detected
