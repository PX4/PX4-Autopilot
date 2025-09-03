#ifndef EKF_GNSS_HEADING_H
#define EKF_GNSS_HEADING_H

#include "sensor.h"

namespace sensor_simulator
{
namespace sensor
{

class GnssHeading: public Sensor
{
public:
	GnssHeading(std::shared_ptr<Ekf> ekf);
	~GnssHeading();

	void setData(const gnss_heading_message &gps_yaw);
	void setHeading(const float heading);
	void setHeadingAccuracy(const float heading_accuracy);

	gnss_heading_message getDefaultGnssHeadingData();

private:
	void send(uint64_t time) override;

	gnss_heading_message _gnss_heading_data{};
};

} // namespace sensor
} // namespace sensor_simulator
#endif // EKF_GNSS_HEADING_H
