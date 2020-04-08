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
	_ekf->set_rangefinder_limits(_min_distance, _max_distance);
}

void RangeFinder::setData(float range_data_meters, int8_t range_quality)
{
	_range_sample.rng = range_data_meters;
	_range_sample.quality = range_quality;
}

void RangeFinder::setLimits(float min_distance_m, float max_distance_m)
{
	_min_distance = min_distance_m;
	_max_distance = max_distance_m;
}

} // namespace sensor
} // namespace sensor_simulator
