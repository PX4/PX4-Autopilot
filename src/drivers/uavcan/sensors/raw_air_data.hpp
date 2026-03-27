#pragma once

#include "sensor_bridge.hpp"

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/air_data/RawAirData.hpp>

#include <uORB/topics/raw_air_data.h>

class UavcanRawAirDataBridge : public UavcanSensorBridgeBase
{
public:
    static const char *const NAME;

    const char *get_name() const override { return NAME; }

    UavcanRawAirDataBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher);

    int init() override;
    void print_status() const override;

private:
    void raw_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &msg);

    using RawCbBinder = uavcan::MethodBinder<UavcanRawAirDataBridge *,
        void (UavcanRawAirDataBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &)>;

    uavcan::Subscriber<uavcan::equipment::air_data::RawAirData, RawCbBinder> _sub_raw;

    float _last_dp_pitot_pa{0.f};
    float _last_static_pressure_pa{0.f};
    float _last_temperature_k{0.f};
};
