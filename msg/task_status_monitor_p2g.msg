# Return current task statue and taget point (Mavros->Pix->GCS)
float32 spray_duration  #spray duration in task set by GCS
uint8   task_status     #task status NO. about Task State Machine, 0~16~30
uint8   loop_value      #loop number of current task, 0~5
float32 target_x        #current target X position, in m
float32 target_y        #current target Y position, in m
float32 target_z        #current target Z position, in m