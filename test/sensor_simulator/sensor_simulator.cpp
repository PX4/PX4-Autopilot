#include "sensor_simulator.h"


SensorSimulator::SensorSimulator(std::shared_ptr<Ekf> ekf):
_ekf{ekf},
_imu{ekf},
_mag{ekf},
_baro{ekf},
_gps{ekf}
{

	// set default sensor rate in Hz
	_imu.setRate(250);
	_mag.setRate(80);
	_baro.setRate(80);
	_gps.setRate(5);

	// set default sensor data
	_imu.setData(Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G},
		     Vector3f{0.0f,0.0f,0.0f});
	_mag.setData(Vector3f{0.2f, 0.0f, 0.4f});
	_baro.setData(122.2f);
	_gps.setData(getDefaultGpsData());

	// start default sensor
	_imu.start();
	_mag.start();
	_baro.start();
}

SensorSimulator::~SensorSimulator()
{

}

gps_message SensorSimulator::getDefaultGpsData()
{
	// setup gps message to reasonable default values
	gps_message gps_data{};
	gps_data.time_usec = 0;
	gps_data.lat = 473566094;
	gps_data.lon = 85190237;
	gps_data.alt = 422056;
	gps_data.yaw = 0.0f;
	gps_data.yaw_offset = 0.0f;
	gps_data.fix_type = 3;
	gps_data.eph = 0.5f;
	gps_data.epv = 0.8f;
	gps_data.sacc = 0.2f;
	gps_data.vel_m_s = 0.0;
	gps_data.vel_ned[0] = 0.0f;
	gps_data.vel_ned[1] = 0.0f;
	gps_data.vel_ned[2] = 0.0f;
	gps_data.vel_ned_valid = 1;
	gps_data.nsats = 16;
	gps_data.pdop = 0.0f;

	return gps_data;
}

void SensorSimulator::run_seconds(float duration_seconds)
{
	run_microseconds( uint32_t(duration_seconds * 1e6f) );
}

void SensorSimulator::run_microseconds(uint32_t duration)
{
	// simulate in 1000us steps
	uint32_t start_time = _time;

	for(;_time < start_time + duration; _time+=1000)
	{
		_imu.update(_time);
		_mag.update(_time);
		_baro.update(_time);
		_gps.update(_time);

		_ekf->update();
	}
}

void SensorSimulator::setImuBias(Vector3f accel_bias, Vector3f gyro_bias)
{
	_imu.setData(Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G} + accel_bias,
		     Vector3f{0.0f,0.0f,0.0f} + gyro_bias);
}
