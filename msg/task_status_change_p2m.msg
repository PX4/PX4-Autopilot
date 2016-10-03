# GCS would change the task status of RUAV by this message (GCS->Pix->Mavros)
float32 spray_duration  #0, 1 or 2 represent spray all number, odd number or even number
uint8 task_status   #task status NO. about Task State Machine, 0~16~30
uint8 loop_value    #loop number of current task, 0~5
