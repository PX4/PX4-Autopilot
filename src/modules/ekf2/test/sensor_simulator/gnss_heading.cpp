#include "gnss_heading.h"

namespace sensor_simulator
{
namespace sensor
{

GnssHeading::GnssHeading(std::shared_ptr<Ekf> ekf) : Sensor(ekf)
{
}

GnssHeading::~GnssHeading()
{
}

void GnssHeading::send(const uint64_t time)
{
	if (!PX4_ISFINITE(_gnss_heading_data.heading)) {
		// if heading is not set, don't send, as in reality, this data wouldn't be published to vehicle_gnss_heading
		return;
	}

	// keep dt in case you want to evolve heading with a rate later
	const float dt = static_cast<float>(time - _gnss_heading_data.time_usec) * 1e-6f;
	(void)dt;

	_gnss_heading_data.time_usec = time;

	// hand the sample to the EKF
	_ekf->setGnssHeadingData(_gnss_heading_data);
}

void GnssHeading::setData(const gnss_heading_message &gps_yaw)
{
	_gnss_heading_data = gps_yaw;
}

void GnssHeading::setHeading(const float heading)
{
	_gnss_heading_data.heading = heading;
}

void GnssHeading::setHeadingAccuracy(const float heading_accuracy)
{
	_gnss_heading_data.heading_accuracy = heading_accuracy;
}

gnss_heading_message GnssHeading::getDefaultGnssHeadingData()
{
	gnss_heading_message gps_yaw{};
	gps_yaw.time_usec = 0;
	gps_yaw.heading = NAN;
	gps_yaw.heading_accuracy = 0.1f;
	return gps_yaw;
}

} // namespace sensor
} // namespace sensor_simulator
