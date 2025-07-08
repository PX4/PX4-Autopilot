# SOARING CONTROLLER WIND ESTIMATE, USED FOR INDI CONTROL

uint64 timestamp			# time since system start (microseconds)

float32[3] wind_estimate                            # WIND ESTIMATE IN ENU FRAME
float32[3] wind_estimate_filtered                   # LP-FILTERED WIND ESTIMATE IN ENU FRAME, USED BY INDI CONTROLLER
float32[3] position                                 # position of the current estimate in the soaring frame
float32    airspeed                                 # current airspeed

bool valid                                          # tell, if estimate is valid for shear estimator
bool lock_params                                    # tell, if the shear estimator shall lock shear param values for soaring
