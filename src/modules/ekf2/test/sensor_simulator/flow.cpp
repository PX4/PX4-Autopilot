#include "flow.h"

namespace sensor_simulator
{
namespace sensor
{

Flow::Flow(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

Flow::~Flow()
{
}

void Flow::send(uint64_t time)
{
	_flow_data.time_us = time;
	_ekf->setOpticalFlowData(_flow_data);
}

void Flow::setData(const flowSample &flow)
{
	_flow_data = flow;

}

flowSample Flow::dataAtRest()
{
	flowSample flow_at_rest;
	flow_at_rest.flow_rate = Vector2f{0.0f, 0.0f};
	flow_at_rest.gyro_rate = Vector3f{0.0f, 0.0f, 0.0f};
	flow_at_rest.quality = 255;
	return flow_at_rest;
}

} // namespace sensor
} // namespace sensor_simulator
