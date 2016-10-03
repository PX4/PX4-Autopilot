# Return current task statue and taget point (Mavros->Pix->GCS)
float32 spray_duration  #spray duration in task set by GCS
uint8   task_status     #task status NO. about Task State Machine, 0~16~30
uint8   loop_value      #loop number of current task, 0~5
float64 target_lat      #current target latitude, in deg
float64 target_lon      #current target longitude, in deg
float32 target_alt      #current target altitude, in m