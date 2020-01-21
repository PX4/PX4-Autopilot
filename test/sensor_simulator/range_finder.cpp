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
	_range_sample.time_us = time;
	_ekf->setRangeData(_range_sample);
}

void RangeFinder::setData(float range_data_meters, int8_t range_quality)
{
	_range_sample.rng = range_data_meters;
	_range_sample.quality = range_quality;
}

} // namespace sensor
} // namespace sensor_simulator
