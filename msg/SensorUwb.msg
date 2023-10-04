# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)

uint32 	sessionid		# UWB SessionID
uint32 	time_offset		# Time between Ranging Rounds in ms
uint32 	counter			# Number of Ranges since last Start of Ranging
uint16 	mac			# MAC adress of Initiator (controller)

uint16 	mac_dest		# MAC adress of Responder (Controlee)
uint16 	status			# status feedback #
uint8 	nlos			# None line of site condition y/n
float32 distance		# distance in m to the UWB receiver


#Angle of arrival, Angle in Degree -60..+60; FOV in both axis is 120 degrees
float32 aoa_azimuth_dev	# Angle of arrival of first incomming RX msg
float32 aoa_elevation_dev	# Angle of arrival of first incomming RX msg
float32 aoa_azimuth_resp	# Angle of arrival of first incomming RX msg at the responder
float32 aoa_elevation_resp	# Angle of arrival of first incomming RX msg at the responder

# Figure of merit for the angle measurements
uint8 aoa_azimuth_fom		# AOA Azimuth FOM
uint8 aoa_elevation_fom		# AOA Elevation FOM
uint8 aoa_dest_azimuth_fom	# AOA Azimuth FOM
uint8 aoa_dest_elevation_fom	# AOA Elevation FOM

# Initiator physical configuration
uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
				# Standard configuration is Antennas facing down and azimuth aligened in forward direction
float32 offset_x		# UWB initiator offset in X axis (NED drone frame)
float32 offset_y		# UWB initiator offset in Y axis (NED drone frame)
float32 offset_z		# UWB initiator offset in Z axis (NED drone frame)
