#include "imu.h"

namespace sensor_simulator
{
namespace sensor
{

Imu::Imu(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Imu::~Imu()
{
}

void Imu::send(uint64_t time)
{
	const float dt = float((time - _time_last_data_sent) * 1.e-6f);
	imuSample imu_sample{};
	imu_sample.time_us = time;
	imu_sample.delta_ang_dt = dt;
	imu_sample.delta_ang = _gyro_data * imu_sample.delta_ang_dt;
	imu_sample.delta_vel_dt = dt;
	imu_sample.delta_vel = _accel_data * imu_sample.delta_vel_dt;

	_ekf->setIMUData(imu_sample);
}

void Imu::setData(const Vector3f& accel, const Vector3f& gyro)
{
	setAccelData(accel);
	setGyroData(gyro);
}

void Imu::setAccelData(const Vector3f& accel)
{
	_accel_data = accel;
}

void Imu::setGyroData(const Vector3f& gyro)
{
	_gyro_data = gyro;
}

} // namespace sensor
} // namespace sensor_simulator
