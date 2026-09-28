# Ultra-wideband (UWB) distance sensor
#
# Distance and angle-of-arrival ranging data from an ultra-wideband (UWB) positioning
# system, such as Pozyx or NXP Rddrone. Published and consumed internally by the UWB
# ranging driver (src/drivers/uwb/uwb_sr150), which parses serial ranging results from
# the sensor module and publishes them on this topic; no other module in the codebase
# currently subscribes to it.

uint64 timestamp # [us] Time since system start

uint32 sessionid # UWB SessionID
uint32 time_offset # Time between Ranging Rounds in ms
uint32 counter # Number of Ranges since last Start of Ranging
uint16 mac # [-] MAC address of Initiator (controller)

uint16 mac_dest # [-] MAC address of Responder (Controlee)
uint16 status # [-] Ranging status code passed through from the UWB module's measurement result
uint8 nlos # [-] Non-line-of-sight condition (y/n)
float32 distance # [m] Distance to the UWB receiver

#Angle of arrival, Angle in Degree -60..+60; FOV in both axis is 120 degrees
float32 aoa_azimuth_dev # [deg] Angle of arrival of first incoming RX msg
float32 aoa_elevation_dev # [deg] Angle of arrival of first incoming RX msg
float32 aoa_azimuth_resp # [deg] Angle of arrival of first incoming RX msg at the responder
float32 aoa_elevation_resp # [deg] Angle of arrival of first incoming RX msg at the responder

# Figure of merit for the angle measurements
uint8 aoa_azimuth_fom # AOA Azimuth Figure of Merit (FOM)
uint8 aoa_elevation_fom # AOA Elevation FOM
uint8 aoa_dest_azimuth_fom # AOA Azimuth FOM
uint8 aoa_dest_elevation_fom # AOA Elevation FOM

# Initiator physical configuration
uint8 orientation # Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. Standard configuration has antennas facing down and azimuth aligned in the forward direction.
float32 offset_x # UWB initiator offset in X axis (NED drone frame)
float32 offset_y # UWB initiator offset in Y axis (NED drone frame)
float32 offset_z # UWB initiator offset in Z axis (NED drone frame)
