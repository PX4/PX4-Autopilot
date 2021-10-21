#include "vio.h"

namespace sensor_simulator
{
namespace sensor
{

Vio::Vio(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

Vio::~Vio()
{
}

void Vio::send(uint64_t time)
{
	_vio_data.time_us = time;
	_ekf->setExtVisionData(_vio_data);
}

void Vio::setData(const extVisionSample &vio_data)
{
	_vio_data = vio_data;
}

void Vio::setVelocityVariance(const Vector3f &velVar)
{
	setVelocityCovariance(matrix::diag(velVar));
}

void Vio::setVelocityCovariance(const Matrix3f &velCov)
{
	_vio_data.velCov = velCov;
}

void Vio::setPositionVariance(const Vector3f &posVar)
{
	_vio_data.posVar = posVar;
}

void Vio::setAngularVariance(float angVar)
{
	_vio_data.angVar = angVar;
}

void Vio::setVelocity(const Vector3f &vel)
{
	_vio_data.vel = vel;
}

void Vio::setPosition(const Vector3f &pos)
{
	_vio_data.pos = pos;
}

void Vio::setOrientation(const Quatf &quat)
{
	_vio_data.quat = quat;
}

void Vio::setVelocityFrameToBody()
{
	_vio_data.vel_frame = velocity_frame_t::BODY_FRAME_FRD;
}

void Vio::setVelocityFrameToLocal()
{
	_vio_data.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;
}

extVisionSample Vio::dataAtRest()
{
	extVisionSample vio_data;
	vio_data.pos = Vector3f{0.0f, 0.0f, 0.0f};;
	vio_data.vel = Vector3f{0.0f, 0.0f, 0.0f};;
	vio_data.quat = Quatf{1.0f, 0.0f, 0.0f, 0.0f};
	vio_data.posVar = Vector3f{0.1f, 0.1f, 0.1f};
	vio_data.velCov = matrix::eye<float, 3>() * 0.1f;
	vio_data.angVar = 0.05f;
	vio_data.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;
	return vio_data;
}

} // namespace sensor
} // namespace sensor_simulator
