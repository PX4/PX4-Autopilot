from pyulog import ULog
import pandas as pd

def getVioData(ulog: ULog) -> pd.DataFrame:

	vehicle_visual_odometry = ulog.get_dataset("vehicle_visual_odometry").data
	vio = pd.DataFrame({'timestamp': vehicle_visual_odometry['timestamp'],
		'sensor' : 'vio',
		'x': vehicle_visual_odometry["position[0]"],
		'y': vehicle_visual_odometry["position[1]"],
		'z': vehicle_visual_odometry["position[2]"],
		'qw': vehicle_visual_odometry["q[0]"],
		'qx': vehicle_visual_odometry["q[1]"],
		'qy': vehicle_visual_odometry["q[2]"],
		'qz': vehicle_visual_odometry["q[3]"],
		'vx': vehicle_visual_odometry["velocity[0]"],
		'vy': vehicle_visual_odometry["velocity[1]"],
		'vz': vehicle_visual_odometry["velocity[2]"]
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

	# The column names have to be unique across all sensors: the tables are merged by
	# column name and the resulting column order defines the order of the values in the
	# csv. 'quality' is already used by the optical flow, reusing it here would append
	# the distance behind the quality and swap the two arguments of
	# _rng.setData(distance, quality) in SensorSimulator::setSingleReplaySample().
	rng = pd.DataFrame()

	for instance in range(3):
		try:
			distance_sensor = ulog.get_dataset("distance_sensor", instance).data
		except (KeyError, IndexError, ValueError):
			# instance not logged
			continue

		rng_instance = pd.DataFrame({'timestamp': distance_sensor['timestamp'],
			'sensor' : 'range',
			'range_distance': distance_sensor["current_distance"],
			'range_quality': distance_sensor["signal_quality"]
			})
		rng = pd.concat([rng, rng_instance], ignore_index=True, sort=False)

	return rng


def getGpsData(ulog: ULog) -> pd.DataFrame:

	vehicle_gps_position = ulog.get_dataset("vehicle_gps_position").data
	# The column order has to match SensorSimulator::setSingleReplaySample(), which reads
	# altitude, latitude, longitude. The loader also rescales the latitude and the
	# longitude by 1e-7 because they used to be logged as scaled integers, so the degrees
	# have to be scaled up by 1e7 here.
	gps = pd.DataFrame({'timestamp': vehicle_gps_position['timestamp'],
		'sensor' : 'gps',
		'alt': vehicle_gps_position["altitude_msl_m"],
		'lat': vehicle_gps_position["latitude_deg"] * 1e7,
		'lon': vehicle_gps_position["longitude_deg"] * 1e7,
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
