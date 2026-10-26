#pragma once

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/dronecan_node_status.h>
#include <drivers/drv_hrt.h>

class MavlinkStreamUavcanNodeStatus : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamUavcanNodeStatus(mavlink);
    }

    static constexpr const char *get_name_static()
    {
        return "UAVCAN_NODE_STATUS";
    }

    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
    }

    const char *get_name() const override
    {
        return get_name_static();
    }

    uint16_t get_id() override
    {
        return get_id_static();
    }

    unsigned get_size() override
    {
        return ORB_MULTI_MAX_INSTANCES *
               (MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN +
                MAVLINK_NUM_NON_PAYLOAD_BYTES);
    }

private:
    explicit MavlinkStreamUavcanNodeStatus(Mavlink *mavlink)
        : MavlinkStream(mavlink)
    {
    }

    static constexpr hrt_abstime STATUS_TIMEOUT_US = 3000000;

    uORB::SubscriptionMultiArray<
        dronecan_node_status_s,
        ORB_MULTI_MAX_INSTANCES
    > _node_status_subs{ORB_ID::dronecan_node_status};

    bool send() override
    {
        bool sent_any = false;
        const hrt_abstime now = hrt_absolute_time();

        constexpr unsigned packet_size =
            MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN +
            MAVLINK_NUM_NON_PAYLOAD_BYTES;

        unsigned tx_budget = _mavlink->get_free_tx_buf();

        for (auto &sub : _node_status_subs) {
            dronecan_node_status_s status{};

            if (!sub.copy(&status)) {
                continue;
            }

            if (status.node_id == 0 || status.node_id > 127) {
                continue;
            }

            if (status.timestamp == 0 ||
                status.timestamp > now ||
                (now - status.timestamp) > STATUS_TIMEOUT_US) {
                continue;
            }

            if (tx_budget < packet_size) {
                break;
            }

            mavlink_uavcan_node_status_t message{};
            message.time_usec = status.timestamp;
            message.uptime_sec = status.uptime_sec;
            message.health = status.health;
            message.mode = status.mode;
            message.sub_mode = status.sub_mode;
            message.vendor_specific_status_code =
                status.vendor_specific_status_code;

            mavlink_message_t packet{};

            mavlink_msg_uavcan_node_status_encode_chan(
                mavlink_system.sysid,
                static_cast<uint8_t>(status.node_id),
                _mavlink->get_channel(),
                &packet,
                &message
            );

            _mavlink_resend_uart(
                _mavlink->get_channel(),
                &packet
            );

            tx_budget -= packet_size;
            sent_any = true;
        }

        return sent_any;
    }
};
