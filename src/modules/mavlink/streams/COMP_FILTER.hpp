#ifndef COMP_FILTER_HPP
#define COMP_FILTER_HPP

#include <uORB/topics/comp_filter_status.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>


class MavlinkStreamCompFilter : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamCompFilter(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkStreamCompFilter::get_name_static();
    }
    static const char *get_name_static()
    {
        return "ATTITUDE";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_ATTITUDE;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    //Subscription to array of uORB battery status instances
    uORB::Subscription _comp_filter_sub{ORB_ID(comp_filter_status)};
    // SubscriptionMultiArray subscription is needed because battery has multiple instances.
    // uORB::Subscription is used to subscribe to a single-instance topic

    /* do not allow top copying this class */
    MavlinkStreamCompFilter(MavlinkStreamCompFilter &);
    MavlinkStreamCompFilter& operator = (const MavlinkStreamCompFilter &);

protected:
    explicit MavlinkStreamCompFilter(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}
    bool send() override
	{
		comp_filter_status_s att;

		if (_comp_filter_sub.update(&att)) {

			mavlink_attitude_t msg{};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = att.comp_angles[0];
			msg.pitch = att.comp_angles[1];
			msg.yaw = att.comp_angles[2];

			msg.rollspeed  = 0.0f;
			msg.pitchspeed = 0.0f;
			msg.yawspeed   = 0.0f;

			mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}

};

#endif