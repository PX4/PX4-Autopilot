#ifndef WINCH_STATUS_CUSTOM_HPP
#define WINCH_STATUS_CUSTOM_HPP

#include "../mavlink_stream.h"
#include <uORB/topics/winch_status.h>
#include <uORB/Subscription.hpp>

class MavlinkStreamWinchStatusCustom : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamWinchStatusCustom(mavlink);
    }

    static constexpr const char *get_name_static() { return "WINCH_STATUS_CUSTOM"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WINCH_STATUS_CUSTOM; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _winch_status_sub.advertised() ?
               MAVLINK_MSG_ID_WINCH_STATUS_CUSTOM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:
    explicit MavlinkStreamWinchStatusCustom(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _winch_status_sub{ORB_ID(winch_status)};

    bool send() override
    {
        winch_status_s status;

        if (_winch_status_sub.update(&status)) {
            mavlink_winch_status_custom_t msg{};

            msg.timestamp = status.timestamp;
            msg.device_address = status.device_address;
            msg.device_id = status.device_id;
            msg.pcb_id = status.pcb_id;
            msg.firmware_version = status.firmware_version;
            msg.system_uptime = status.system_uptime;
            msg.total_uptime = status.total_uptime;
            msg.total_rope_used = status.total_rope_used;
            msg.winch_state = status.winch_state;
            msg.motor_state = status.motor_state;
            msg.hook_state = status.hook_state;
            msg.release_module_state = status.release_module_state;
            msg.input_voltage = status.input_voltage;
            msg.motor_bus_current = status.motor_bus_current;
            msg.motor_phase_current = status.motor_phase_current;
            msg.release_module_voltage = status.release_module_voltage;
            msg.motor_speed = status.motor_speed;
            msg.winch_speed = status.winch_speed;
            msg.rope_length = status.rope_length;
            msg.load_weight = status.load_weight;
            msg.fuse_battery_voltage = status.fuse_battery_voltage;
            msg.fuse_battery_state = status.fuse_battery_state;
            msg.total_fuse_count = status.total_fuse_count;
            msg.total_overload_count = status.total_overload_count;
            msg.rope_direction = status.rope_direction;
            msg.rope_max_angle = status.rope_max_angle;
            msg.rope_max_angle_reverse = status.rope_max_angle_reverse;
            msg.rope_swing_frequency = status.rope_swing_frequency;
            msg.rope_angular_velocity = status.rope_angular_velocity;
            msg.system_healthy = status.system_healthy;
            msg.comm_link_status = status.comm_link_status;

            mavlink_msg_winch_status_custom_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }

        return false;
    }
};

#endif // WINCH_STATUS_CUSTOM_HPP
