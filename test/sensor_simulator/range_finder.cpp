#include "range_finder.h"

namespace sensor_simulator
{
namespace sensor
{

RangeFinder::RangeFinder(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

RangeFinder::~RangeFinder()
{
}

void RangeFinder::send(uint64_t time)
{
	_ekf->setRangeData(time, _range_data, _range_quality);
}

void RangeFinder::setData(float range_data_meters, int8_t range_quality)
{
	_range_data = range_data_meters;
	_range_quality = range_quality;
}

} // namespace sensor
} // namespace sensor_simulator
