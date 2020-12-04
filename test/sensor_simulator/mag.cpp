#include "mag.h"

namespace sensor_simulator
{
namespace sensor
{

Mag::Mag(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Mag::~Mag()
{
}

void Mag::send(uint64_t time)
{
	_ekf->setMagData(magSample{time, _mag_data});
}

void Mag::setData(const Vector3f& mag)
{
	_mag_data = mag;
}

} // namespace sensor
} // namespace sensor_simulator
