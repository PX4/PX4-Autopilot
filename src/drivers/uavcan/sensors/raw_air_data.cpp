#include <drivers/drv_hrt.h>

#include "raw_air_data.hpp"

const char *const UavcanRawAirDataBridge::NAME = "raw_air_data";

UavcanRawAirDataBridge::UavcanRawAirDataBridge(uavcan::INode &node,
                                               NodeInfoPublisher *node_info_publisher) :
    UavcanSensorBridgeBase("uavcan_raw_air_data", ORB_ID(raw_air_data), node_info_publisher),
    _sub_raw(node)
{
}

int UavcanRawAirDataBridge::init()
{
    const int res = _sub_raw.start(RawCbBinder(this, &UavcanRawAirDataBridge::raw_sub_cb));

    if (res < 0) {
        DEVICE_LOG("failed to start RawAirData sub: %d", res);
        return res;
    }

    return 0;
}

void UavcanRawAirDataBridge::print_status() const
{
    DEVICE_LOG("raw_air_data: dp_pitot=%.3f Pa, Pstatic=%.3f Pa, T=%.3f K",
               (double)_last_dp_pitot_pa,
               (double)_last_static_pressure_pa,
               (double)_last_temperature_k);
}

void UavcanRawAirDataBridge::raw_sub_cb(
    const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &msg)
{
    _last_static_pressure_pa = msg.static_pressure;
    _last_dp_pitot_pa        = msg.differential_pressure;
    _last_temperature_k      = msg.static_air_temperature;

    raw_air_data_s report{};
    report.timestamp          = hrt_absolute_time();
    report.dp_pitot_pa        = _last_dp_pitot_pa;
    report.static_pressure_pa = _last_static_pressure_pa;
    report.temperature_k      = _last_temperature_k;

    publish(msg.getSrcNodeID().get(), &report);
}
