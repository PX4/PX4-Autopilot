from pyulog import ULog
import argparse
import pandas as pd
import csv
import convertULogToSensorData as util


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
	except Exception as e:
		print("Could not read ulog file: {}: {}".format(type(e).__name__, e))
		exit(-1)

	try:
		imu = util.getImuData(ulog)
		print("IMU data detected")
		table = pd.concat([table, imu], ignore_index=True, sort=False)
	except Exception as e:
		print("IMU data not detected: {}: {}".format(type(e).__name__, e))

	try:
		mag = util.getMagnetometerData(ulog)
		print("Mag data detected")
		table = pd.concat([table, mag], ignore_index=True, sort=False)
	except Exception as e:
		print("Mag data not detected: {}: {}".format(type(e).__name__, e))

	try:
		baro = util.getBarometerData(ulog)
		print("Baro data detected")
		table = pd.concat([table, baro], ignore_index=True, sort=False)
	except Exception as e:
		print("Baro data not detected: {}: {}".format(type(e).__name__, e))

	try:
		gps = util.getGpsData(ulog)
		print("GPS data detected")
		table = pd.concat([table, gps], ignore_index=True, sort=False)
	except Exception as e:
		print("GPS data not detected: {}: {}".format(type(e).__name__, e))

	try:
		airspeed = util.getAirspeedData(ulog)
		print("Airspeed data detected")
		table = pd.concat([table, airspeed], ignore_index=True, sort=False)
	except Exception as e:
		print("Airspeed data not detected: {}: {}".format(type(e).__name__, e))

	# The optical flow has to be added before the range finder: the column order of the
	# merged table defines the order of the values in the csv, which the C++ loader reads
	# positionally.
	try:
		flow = util.getOpticalFlowData(ulog)
		print("Flow data detected")
		table = pd.concat([table, flow], ignore_index=True, sort=False)
	except Exception as e:
		print("Flow data not detected: {}: {}".format(type(e).__name__, e))

	try:
		range = util.getRangeFinderData(ulog)
		print("Range data detected")
		table = pd.concat([table, range], ignore_index=True, sort=False)
	except Exception as e:
		print("Range data not detected: {}: {}".format(type(e).__name__, e))

	try:
		vio = util.getVioData(ulog)
		print("VIO data detected")
		table = pd.concat([table, vio], ignore_index=True, sort=False)
	except Exception as e:
		print("VIO data not detected: {}: {}".format(type(e).__name__, e))

	try:
		land = util.getVehicleLandingStatus(ulog)
		print("Landing data detected")
		table = pd.concat([table, land], ignore_index=True, sort=False)
	except Exception as e:
		print("Landing data not detected: {}: {}".format(type(e).__name__, e))

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
	except Exception as e:
		print("Could not write to specified output file: {}: {}".format(type(e).__name__, e))


if __name__ == '__main__':
	main()
