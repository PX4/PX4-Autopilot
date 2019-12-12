#include "Gps.h"

Gps::Gps(Ekf* ekf):Sensor(ekf)
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

void Gps::setData(gps_message gps)
{
	_gps_data = gps;
}
