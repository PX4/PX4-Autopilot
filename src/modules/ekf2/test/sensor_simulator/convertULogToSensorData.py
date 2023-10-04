from pyulog import ULog
import pandas as pd

def getVioData(ulog: ULog) -> pd.DataFrame:

	vehicle_visual_odometry = ulog.get_dataset("vehicle_visual_odometry").data
	vio = pd.DataFrame({'timestamp': vehicle_visual_odometry['timestamp'],
		'sensor' : 'vio',
		'x': vehicle_visual_odometry["x"],
		'y': vehicle_visual_odometry["y"],
		'z': vehicle_visual_odometry["z"],
		'qw': vehicle_visual_odometry["q[0]"],
		'qx': vehicle_visual_odometry["q[1]"],
		'qy': vehicle_visual_odometry["q[2]"],
		'qz': vehicle_visual_odometry["q[3]"],
		'vx': vehicle_visual_odometry["vx"],
		'vy': vehicle_visual_odometry["vy"],
		'vz': vehicle_visual_odometry["vz"]
		})
	return vio


def getOpticalFlowData(ulog: ULog) -> pd.DataFrame:

	optical_flow = ulog.get_dataset("vehicle_optical_flow").data
	flow = pd.DataFrame({'timestamp': optical_flow['timestamp'],
	'sensor' : 'flow',
	'pixel_flow_x': optical_flow["pixel_flow[0]"],
	'pixel_flow_y': optical_flow["pixel_flow[1]"],
	'delta_angle_x': optical_flow["delta_angle[0]"],
	'delta_angle_y': optical_flow["delta_angle[1]"],
	'delta_angle_z': optical_flow["delta_angle[2]"],
	'quality': optical_flow["quality"]
		})
	return flow


def getAirspeedData(ulog: ULog) -> pd.DataFrame:

	airspeed = ulog.get_dataset("airspeed").data
	airspeed = pd.DataFrame({'timestamp': airspeed['timestamp'],
		'sensor' : 'airspeed',
		'true_as': airspeed["true_airspeed_m_s"],
		'indicated_as': airspeed["indicated_airspeed_m_s"]
		})
	return airspeed


def getRangeFinderData(ulog: ULog) -> pd.DataFrame:

	range = pd.DataFrame()
	try:
		range_0 = ulog.get_dataset("distance_sensor", 0).data
		rng_0 = pd.DataFrame({'timestamp': range_0['timestamp'],
			'sensor' : 'range',
			'data': range_0["current_distance"],
			'quality': range_0["signal_quality"]
			})
		range = pd.concat([range, rng_0], ignore_index=True, sort=False)
	except:
		pass

	try:
		range_1 = ulog.get_dataset("distance_sensor", 1).data
		rng_1 = pd.DataFrame({'timestamp': range_1['timestamp'],
			'sensor' : 'range',
			'data': range_1["current_distance"],
			'quality': range_1["signal_quality"]
			})
		range = pd.concat([range, rng_1], ignore_index=True, sort=False)
	except:
		pass

	try:
		range_2 = ulog.get_dataset("distance_sensor", 2).data
		rng_2 = pd.DataFrame({'timestamp': range_2['timestamp'],
			'sensor' : 'range',
			'data': range_2["current_distance"],
			'quality': range_2["signal_quality"]
			})
		range = pd.concat([range, rng_2], ignore_index=True, sort=False)
	except:
		pass

	return range


def getGpsData(ulog: ULog) -> pd.DataFrame:

	vehicle_gps_position = ulog.get_dataset("vehicle_gps_position").data
	gps = pd.DataFrame({'timestamp': vehicle_gps_position['timestamp'],
		'sensor' : 'gps',
		'alt': vehicle_gps_position["alt"],
		'lon': vehicle_gps_position["lon"],
		'lat': vehicle_gps_position["lat"],
		'vel_N': vehicle_gps_position["vel_n_m_s"],
		'vel_E': vehicle_gps_position["vel_e_m_s"],
		'vel_D': vehicle_gps_position["vel_d_m_s"],
		})
	return gps


def getBarometerData(ulog: ULog) -> pd.DataFrame:

	vehicle_air_data = ulog.get_dataset("vehicle_air_data").data
	baro = pd.DataFrame({'timestamp': vehicle_air_data['timestamp'],
		'sensor' : 'baro',
		'baro_alt_meter': vehicle_air_data["baro_alt_meter"]})
	return baro


def getMagnetometerData(ulog: ULog) -> pd.DataFrame:

	vehicle_magnetometer = ulog.get_dataset("vehicle_magnetometer").data
	mag = pd.DataFrame({'timestamp': vehicle_magnetometer['timestamp'],
		'sensor' : 'mag',
		'magnetometer_ga[0]': vehicle_magnetometer["magnetometer_ga[0]"],
		'magnetometer_ga[1]': vehicle_magnetometer["magnetometer_ga[1]"],
		'magnetometer_ga[2]': vehicle_magnetometer["magnetometer_ga[2]"]})
	return mag


def getImuData(ulog: ULog) -> pd.DataFrame:

	sensor_combined = ulog.get_dataset("sensor_combined").data
	imu = pd.DataFrame({'timestamp': sensor_combined['timestamp'],
		'sensor' : 'imu',
		'accel_m_s2[0]': sensor_combined["accelerometer_m_s2[0]"],
		'accel_m_s2[1]': sensor_combined["accelerometer_m_s2[1]"],
		'accel_m_s2[2]': sensor_combined["accelerometer_m_s2[2]"],
		'gyro_rad[0]': sensor_combined["gyro_rad[0]"],
		'gyro_rad[1]': sensor_combined["gyro_rad[1]"],
		'gyro_rad[2]': sensor_combined["gyro_rad[2]"]})
	return imu

def getVehicleLandingStatus(ulog: ULog) -> pd.DataFrame:
	vehicle_land_detected = ulog.get_dataset("vehicle_land_detected").data
	land = pd.DataFrame({'timestamp': vehicle_land_detected['timestamp'],
		'sensor' : 'landed',
		'landed': vehicle_land_detected["landed"]})
	return land
