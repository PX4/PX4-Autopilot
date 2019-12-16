#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

Gps::Gps(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Gps::~Gps()
{
}

void Gps::send(uint32_t time)
{
	_gps_data.time_usec = time;
	_ekf->setGpsData(time, _gps_data);
}

void Gps::setData(const gps_message& gps)
{
	_gps_data = gps;
}

} // namespace sensor
} // namespace sensor_simulator::sensor
