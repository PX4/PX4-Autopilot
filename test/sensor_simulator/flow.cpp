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
	_ekf->setOpticalFlowData(time, &_flow_data);
}

void Flow::setData(const flow_message& flow)
{
	_flow_data = flow;

}

flow_message Flow::dataAtRest()
{
	flow_message _flow_at_rest;
	_flow_at_rest.dt = 20000;
	_flow_at_rest.flowdata = Vector2f{0.0f, 0.0f};
	_flow_at_rest.gyrodata = Vector3f{0.0f, 0.0f, 0.0f};
	_flow_at_rest.quality = 255;
	return _flow_at_rest;
}

} // namespace sensor
} // namespace sensor_simulator
