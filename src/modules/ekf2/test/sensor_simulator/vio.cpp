#include "vio.h"

namespace sensor_simulator
{
namespace sensor
{

Vio::Vio(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
	_vio_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
	_vio_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
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
	_vio_data.velocity_var = velVar;
}

void Vio::setPositionVariance(const Vector3f &posVar)
{
	_vio_data.position_var = posVar;
}

void Vio::setAngularVariance(float angVar)
{
	_vio_data.orientation_var(2) = angVar;
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
	_vio_data.vel_frame = VelocityFrame::BODY_FRAME_FRD;
}

void Vio::setVelocityFrameToLocalFRD()
{
	_vio_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
}

void Vio::setVelocityFrameToLocalNED()
{
	_vio_data.vel_frame = VelocityFrame::LOCAL_FRAME_NED;
}

void Vio::setPositionFrameToLocalNED()
{
	_vio_data.pos_frame = PositionFrame::LOCAL_FRAME_NED;
}

void Vio::setPositionFrameToLocalFRD()
{
	_vio_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
}

extVisionSample Vio::dataAtRest()
{
	extVisionSample vio_data;
	vio_data.pos = Vector3f{0.0f, 0.0f, 0.0f};
	vio_data.vel = Vector3f{0.0f, 0.0f, 0.0f};
	vio_data.quat = Quatf{1.0f, 0.0f, 0.0f, 0.0f};
	vio_data.position_var = Vector3f{0.1f, 0.1f, 0.1f};
	vio_data.velocity_var = Vector3f{0.1f, 0.1f, 0.1f};
	vio_data.orientation_var(2) = 0.05f;
	vio_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
	vio_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
	return vio_data;
}

} // namespace sensor
} // namespace sensor_simulator
