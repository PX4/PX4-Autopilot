#include "px4/rc_channels.h"
#include "platforms/px4_message.h"

#pragma once
namespace px4
{

class  px4_rc_channels :
	public PX4Message<rc_channels>
{
public:
	px4_rc_channels() :
		PX4Message<rc_channels>()
	{}

	px4_rc_channels(rc_channels msg) :
		PX4Message<rc_channels>(msg)
	{}

	~px4_rc_channels() {}

	static PX4TopicHandle handle() {return "rc_channels";}
};

}
