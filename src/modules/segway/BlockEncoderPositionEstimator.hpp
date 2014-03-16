#pragma once

#include <controllib/uorb/blocks.hpp>
#include <uORB/topics/encoders.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>

using namespace control;

class BlockEncoderPositionEstimator : public SuperBlock
{
public:
	BlockEncoderPositionEstimator();
	void update();
private:
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Publication<vehicle_local_position_s> _localPos;
	uORB::Publication<vehicle_global_position_s> _pos;
	uORB::Subscription<encoders_s> _encoders;
	BlockParamFloat _rWheel;
	BlockParamFloat _pulsesPerRev;
	struct pollfd _poll;
	uint64_t _timeStamp;
	struct map_projection_reference_s _pos_ref;
};
