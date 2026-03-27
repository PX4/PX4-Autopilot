#include <drivers/drv_hrt.h>

#include "fivehole_probe.hpp"

const char *const UavcanFiveholeProbeBridge::NAME = "fivehole_probe";

UavcanFiveholeProbeBridge::UavcanFiveholeProbeBridge(uavcan::INode &node,
                                                     NodeInfoPublisher *node_info_publisher) :
    UavcanSensorBridgeBase("uavcan_fivehole_probe", ORB_ID(fivehole_probe), node_info_publisher),
    _sub_aoa(node),
    _sub_sideslip(node)
{
}

int UavcanFiveholeProbeBridge::init()
{
    int res = _sub_aoa.start(AOACbBinder(this, &UavcanFiveholeProbeBridge::aoa_sub_cb));

    if (res < 0) {
        DEVICE_LOG("failed to start AngleOfAttack sub: %d", res);
        return res;
    }

    res = _sub_sideslip.start(SideslipCbBinder(this, &UavcanFiveholeProbeBridge::sideslip_sub_cb));

    if (res < 0) {
        DEVICE_LOG("failed to start Sideslip sub: %d", res);
        return res;
    }

    return 0;
}

void UavcanFiveholeProbeBridge::print_status() const
{
    DEVICE_LOG("fivehole_probe: aoa_dp=%.3f Pa, sideslip_dp=%.3f Pa",
               (double)_last_aoa_dp_pa, (double)_last_sideslip_dp_pa);
}

void UavcanFiveholeProbeBridge::aoa_sub_cb(
    const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::AngleOfAttack> &msg)
{
    // Sponsor interim: raw AOA differential pressure (Pa) is in aoa_variance
    _last_aoa_dp_pa = msg.aoa_variance;

    fivehole_probe_s report{};
    report.timestamp      = hrt_absolute_time();
    report.aoa_dp_pa      = _last_aoa_dp_pa;
    report.sideslip_dp_pa = _last_sideslip_dp_pa;

    publish(msg.getSrcNodeID().get(), &report);
}

void UavcanFiveholeProbeBridge::sideslip_sub_cb(
    const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::Sideslip> &msg)
{
    // Sponsor interim: raw sideslip differential pressure (Pa) is in sideslip_angle_variance
    _last_sideslip_dp_pa = msg.sideslip_angle_variance;

    fivehole_probe_s report{};
    report.timestamp      = hrt_absolute_time();
    report.aoa_dp_pa      = _last_aoa_dp_pa;
    report.sideslip_dp_pa = _last_sideslip_dp_pa;

    publish(msg.getSrcNodeID().get(), &report);
}
