# Gps quality indicators

uint64 timestamp		# time since system start (microseconds)

uint8 quality_corrections       # Abstract corrections quality from 0 to 10, or 255 if not available
uint8 quality_receiver          # Abstract overall receiver operating status from 0 to 10, or 255 if not available
uint8 quality_gnss_signals      # Abstract quality of GNSS signals from 0 to 10, or 255 if not available
uint8 quality_post_processing   # Abstract expected post processing quality from 0 to 10, or 255 if not available
