#include "Sensor.h"

Sensor::Sensor(Ekf* ekf)
{
	_ekf = ekf;
}

Sensor::~Sensor()
{
}

void Sensor::update(uint32_t time)
{
	if(should_send(time))
	{
		send(time);
	}
}

bool Sensor::should_send(uint32_t time)
{
	return _is_running && is_time_to_send(time);
}

bool Sensor::is_time_to_send(uint32_t time)
{
	return (time >= _time_last_data_sent) && ((time - _time_last_data_sent) >= _update_period);
}
