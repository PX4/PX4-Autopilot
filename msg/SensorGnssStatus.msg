# Gnss quality indicators

uint64 timestamp		# time since system start (microseconds)
uint32 device_id                # unique device ID for the sensor that does not change between power cycles

bool  quality_available         # Set to true if quality indicators are available
uint8 quality_corrections       # Corrections quality from 0 to 10, or 255 if not available
uint8 quality_receiver          # Overall receiver operating status from 0 to 10, or 255 if not available
uint8 quality_gnss_signals      # Quality of GNSS signals from 0 to 10, or 255 if not available
uint8 quality_post_processing   # Expected post processing quality from 0 to 10, or 255 if not available
