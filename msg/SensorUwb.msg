# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)
uint32 	counter			# Number of Ranges since last Start of Ranging
uint32 	sessionid		# UWB SessionID
uint32 	time_offset		# Time between Ranging Rounds in ms
uint16 	mac			# MAC adress of Initiator

uint16 	status			# status feedback #
uint8 	nlos			# Visual line of sight yes/no
float32 test
float32 distance		# distance in m to the UWB receiver

#Angle of arrival, Angle in Degree -70..+70
float32 aoa_azimuth_dev	# Angle of arrival of first incomming RX msg
float32 aoa_elevation_dev	# Angle of arrival of first incomming RX msg
float32 aoa_azimuth_resp	# Angle of arrival of first incomming RX msg at the responder
float32 aoa_elevation_resp	# Angle of arrival of first incomming RX msg at the responder

# GPS:
#GPS position of target_pos of the UWB system
# float32[4] gps_data #Latitude, Longitude, Altitude, and Yaw Offset to True North  #
# TODO: Loic does this belong in this message anymore? It should be completely in LTE driver but the LTE references this from the sensor_uwb message?
float32[3] position		# Position of the Landing point in relation to the Drone (x,y,z in Meters NED)
