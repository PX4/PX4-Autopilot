#include "Mag.h"

Mag::Mag(Ekf* ekf):Sensor(ekf)
{
}

Mag::~Mag()
{
}

void Mag::send(uint32_t time)
{
	float mag[3];
	_mag_data.copyTo(mag);
	_ekf->setMagData(time,mag);
	_time_last_data_sent = time;
}

void Mag::setData(Vector3f mag)
{
	_mag_data = mag;
}
