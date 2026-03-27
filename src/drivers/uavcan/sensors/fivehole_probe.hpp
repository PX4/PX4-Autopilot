#pragma once

#include "sensor_bridge.hpp"

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/air_data/AngleOfAttack.hpp>
#include <uavcan/equipment/air_data/Sideslip.hpp>

#include <uORB/topics/fivehole_probe.h>

class UavcanFiveholeProbeBridge : public UavcanSensorBridgeBase
{
public:
    static const char *const NAME;

    const char *get_name() const override { return NAME; }

    UavcanFiveholeProbeBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher);

    int init() override;
    void print_status() const override;

private:
    void aoa_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::AngleOfAttack> &msg);
    void sideslip_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::Sideslip> &msg);

    typedef uavcan::MethodBinder<UavcanFiveholeProbeBridge *,
        void (UavcanFiveholeProbeBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::AngleOfAttack> &)>
        AOACbBinder;

    typedef uavcan::MethodBinder<UavcanFiveholeProbeBridge *,
        void (UavcanFiveholeProbeBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::Sideslip> &)>
        SideslipCbBinder;

    uavcan::Subscriber<uavcan::equipment::air_data::AngleOfAttack, AOACbBinder> _sub_aoa;
    uavcan::Subscriber<uavcan::equipment::air_data::Sideslip, SideslipCbBinder> _sub_sideslip;

    float _last_aoa_dp_pa{0.f};
    float _last_sideslip_dp_pa{0.f};
};
