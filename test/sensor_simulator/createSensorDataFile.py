from pyulog import ULog
import argparse
import pandas as pd
import csv
import convertULogToSensorData as util

path = "/home/kamil/Documents/QGroundControl/Logs/iris_vision.ulg"
output_path = "/home/kamil/Documents/QGroundControl/Logs/sensor_data_iris_vision.csv"


def get_arguments():
	"""
	parses the command line arguments
	:return:
	"""
	parser = argparse.ArgumentParser(
	description='Create a csv file with sensor data values in chronological order from a single'
			'ulog file.')
	parser.add_argument('input_file', metavar='file.ulg', help='ULog input file path')
	parser.add_argument('output_file', metavar='file.ulg', help='csv output file path')
	return parser.parse_args()

def main() -> None:
	"""
	main entry point
	:return:
	"""
	args = get_arguments()

	table = pd.DataFrame()

	try:
		ulog = ULog(args.input_file)
	except:
		print("Could not find ulog file")
		exit(-1)

	try:
		imu = util.getImuData(ulog)
		print("IMU data detected")
		table = pd.concat([table, imu], ignore_index=True, sort=False)
	except:
		print("IMU data not detected")

	try:
		mag = util.getMagnetometerData(ulog)
		print("Mag data detected")
		table = pd.concat([table, mag], ignore_index=True, sort=False)
	except:
		print("Mag data not detected")

	try:
		baro = util.getBarometerData(ulog)
		print("Baro data detected")
		table = pd.concat([table, baro], ignore_index=True, sort=False)
	except:
		print("Baro data not detected")

	try:
		gps = util.getGpsData(ulog)
		print("GPS data detected")
		table = pd.concat([table, gps], ignore_index=True, sort=False)
	except:
		print("GPS data not detected")

	try:
		airspeed = util.getAirspeedData(ulog)
		print("Airspeed data detected")
		table = pd.concat([table, airspeed], ignore_index=True, sort=False)
	except:
		print("Airspeed data not detected")

	try:
		flow = util.getOpticalFlowData(ulog)
		print("Flow data detected")
		table = pd.concat([table, flow], ignore_index=True, sort=False)
	except:
		print("Flow data not detected")

	try:
		range = util.getRangeFinderData(ulog)
		print("Range data detected")
		table = pd.concat([table, range], ignore_index=True, sort=False)
	except:
		print("Range data not detected")

	try:
		vio = util.getVioData(ulog)
		print("VIO data detected")
		table = pd.concat([table, vio], ignore_index=True, sort=False)
	except:
		print("VIO data not detected")

	try:
		land = util.getVehicleLandingStatus(ulog)
		print("Landing data detected")
		table = pd.concat([table, land], ignore_index=True, sort=False)
	except:
		print("Landing data not detected")

	table = table.sort_values('timestamp', axis=0, ascending=True)
	table['timestamp'] = table['timestamp'] - table['timestamp'].iloc[0]
	# remove the first 0.5 seconds of data to be robust against faulty initialized data
	table = table[table.timestamp > 500000]
	table.timestamp = table.timestamp - 500000

	try:
		table.to_csv(args.output_file, index=None, header=None)

		# post processing remove empty cells from csv
		result = []
		with open(args.output_file, "r") as in_file:
			reader = csv.reader(in_file)
			result = [[item for item in row if item != ''] for row in reader]

		with open(args.output_file, "w") as out_file:
			csv_writer = csv.writer(out_file)
			csv_writer.writerows(result)
	except:
		print("Could not write to specified output file")


if __name__ == '__main__':
	main()
