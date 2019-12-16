#include "Imu.h"

namespace sensor_simulator::sensor
{

Imu::Imu(Ekf* ekf):Sensor(ekf)
{
}

Imu::~Imu()
{
}

void Imu::send(uint32_t time)
{
	// fill imu sample with stored data
	imuSample imu_sample;
	imu_sample.time_us = time;
	imu_sample.delta_ang_dt = (time - _time_last_data_sent) * 1.e-6f;
	imu_sample.delta_ang = _gyro_data * imu_sample.delta_ang_dt;
	imu_sample.delta_vel_dt = (time - _time_last_data_sent) * 1.e-6f;
	imu_sample.delta_vel = _accel_data * imu_sample.delta_vel_dt;

	_ekf->setIMUData(imu_sample);
	_time_last_data_sent = time;
}

void Imu::setData(Vector3f accel, Vector3f gyro)
{
	setAccelData(accel);
	setGyroData(gyro);
}

void Imu::setAccelData(Vector3f accel)
{
	_accel_data = accel;
}

void Imu::setGyroData(Vector3f gyro)
{
	_gyro_data = gyro;
}

} // namespace sensor_simulator::sensor
