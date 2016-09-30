# GCS would change the task status of RUAV by this message (GCS->Pix->Mavros)
float32 spray_duration  #spray duration in task set by GCS
uint8 task_status       #task status NO. about Task State Machine, 0~16~30
uint8 loop_value        #loop number of current task, 0~5
