#include "flow.h"

namespace sensor_simulator
{
namespace sensor
{

Flow::Flow(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Flow::~Flow()
{
}

void Flow::send(uint64_t time)
{
	_flow_data.dt = time - _time_last_data_sent;
	_flow_data.time_us = time;
	_ekf->setOpticalFlowData(_flow_data);
}

void Flow::setData(const flowSample& flow)
{
	_flow_data = flow;

}

flowSample Flow::dataAtRest()
{
	flowSample _flow_at_rest;
	_flow_at_rest.dt = 0.02f;
	_flow_at_rest.flow_xy_rad = Vector2f{0.0f, 0.0f};
	_flow_at_rest.gyro_xyz = Vector3f{0.0f, 0.0f, 0.0f};
	_flow_at_rest.quality = 255;
	return _flow_at_rest;
}

} // namespace sensor
} // namespace sensor_simulator
