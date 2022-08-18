# SOARING CONTROLLER WIND ESTIMATE, USED FOR INDI CONTROL

uint64 timestamp			# time since system start (microseconds)

float32[3] wind_estimate                            # WIND ESTIMATE IN ENU FRAME
float32[3] wind_estimate_filtered                   # LP-FILTERED WIND ESTIMATE IN ENU FRAME, USED BY INDI CONTROLLER
float32[3] position                                 # position of the current estimate in the soaring frame

bool valid                                          # tell, if estimate is valid for shear estimator
