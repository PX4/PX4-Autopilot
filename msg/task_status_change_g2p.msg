# GCS would change the task status of RUAV by this message (GCS->Pix->Mavros)
uint8 num_odd_even  #0, 1 or 2 represent spray all number, odd number or even number
uint8 task_status   #task status NO. about Task State Machine, 0~16~30
uint8 loop_value    #loop number of current task, 0~5
