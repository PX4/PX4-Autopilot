# SOARING CONTROLLER WIND ESTIMATE, USED FOR INDI CONTROL

uint64 timestamp			# time since system start (microseconds)

float32[3] wind_estimate                            # WIND ESTIMATE IN ENU FRAME
float32[3] wind_estimate_filtered                   # LP-FILTERED WIND ESTIMATE IN ENU FRAME, USED BY INDI CONTROLLER
