#include "gnss_yaw.h"

namespace sensor_simulator
{
namespace sensor
{

GnssYaw::GnssYaw(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

GnssYaw::~GnssYaw()
{
}

void GnssYaw::send(const uint64_t time)
{
	if (!PX4_ISFINITE(_gnss_yaw_data.yaw)) {
		return;
	}

	_gnss_yaw_data.time_us = time - kGnssYawDelayUs;
	_ekf->setGnssYawData(_gnss_yaw_data);
}

void GnssYaw::setYaw(const float yaw)
{
	_gnss_yaw_data.yaw = yaw;
}

void GnssYaw::setYawOffset(const float yaw_offset)
{
	_gnss_yaw_data.yaw_offset = yaw_offset;
}

void GnssYaw::setYawAccuracy(const float yaw_acc)
{
	_gnss_yaw_data.yaw_acc = yaw_acc;
}

} // namespace sensor
} // namespace sensor_simulator
