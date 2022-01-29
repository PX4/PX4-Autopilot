#include "sensor.h"

namespace sensor_simulator
{

Sensor::Sensor(std::shared_ptr<Ekf> ekf): _ekf{ekf}
{
}

Sensor::~Sensor()
{
}

void Sensor::update(uint64_t time)
{
	if (should_send(time)) {
		send(time);
		_time_last_data_sent = time;
	}
}

bool Sensor::should_send(uint64_t time) const
{
	return _is_running && is_time_to_send(time);
}

bool Sensor::is_time_to_send(uint64_t time) const
{
	return (time >= _time_last_data_sent) && ((time - _time_last_data_sent) >= _update_period);
}

} // namespace sensor_simulator
