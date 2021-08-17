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

	optical_flow = ulog.get_dataset("optical_flow").data
	flow = pd.DataFrame({'timestamp': optical_flow['timestamp'],
	'sensor' : 'flow',
	'pixel_flow_x_integral': optical_flow["pixel_flow_x_integral"],
	'pixel_flow_y_integral': optical_flow["pixel_flow_y_integral"],
	'gyro_x_rate_integral': optical_flow["gyro_x_rate_integral"],
	'gyro_y_rate_integral': optical_flow["gyro_y_rate_integral"],
	'gyro_z_rate_integral': optical_flow["gyro_z_rate_integral"],
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

	vehicle_imu = ulog.get_dataset("vehicle_imu").data
	imu = pd.DataFrame({'timestamp': vehicle_imu['timestamp'],
		'sensor' : 'imu',
		'delta_velocity[0]': vehicle_imu["delta_velocity[0]"],
		'delta_velocity[1]': vehicle_imu["delta_velocity[1]"],
		'delta_velocity[2]': vehicle_imu["delta_velocity[2]"],
		'delta_angle[0]': vehicle_imu["delta_angle[0]"],
		'delta_angle[1]': vehicle_imu["delta_angle[1]"],
		'delta_angle[2]': vehicle_imu["delta_angle[2]"]})
	return imu

def getVehicleLandingStatus(ulog: ULog) -> pd.DataFrame:
	vehicle_land_detected = ulog.get_dataset("vehicle_land_detected").data
	land = pd.DataFrame({'timestamp': vehicle_land_detected['timestamp'],
		'sensor' : 'landed',
		'landed': vehicle_land_detected["landed"]})
	return land
