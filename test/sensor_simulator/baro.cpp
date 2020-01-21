#include "baro.h"

namespace sensor_simulator
{
namespace sensor
{

Baro::Baro(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Baro::~Baro()
{
}

void Baro::send(uint64_t time)
{
	const baroSample baro_sample {_baro_data, time};
	_ekf->setBaroData(baro_sample);
}

void Baro::setData(float baro)
{
	_baro_data = baro;
}

} // namespace sensor
} // namespace sensor_simulator
