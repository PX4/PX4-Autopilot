/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/build_config.hpp>
#if !UAVCAN_TINY

#include <uavcan/protocol/transport_stats_provider.hpp>

namespace uavcan
{

void TransportStatsProvider::handleGetTransportStats(const protocol::GetTransportStats::Request&,
                                                     protocol::GetTransportStats::Response& resp) const
{
    const TransferPerfCounter& perf = srv_.getNode().getDispatcher().getTransferPerfCounter();
    resp.transfer_errors = perf.getErrorCount();
    resp.transfers_tx = perf.getTxTransferCount();
    resp.transfers_rx = perf.getRxTransferCount();

    const CanIOManager& canio = srv_.getNode().getDispatcher().getCanIOManager();
    for (uint8_t i = 0; i < canio.getNumIfaces(); i++)
    {
        const CanIfacePerfCounters can_perf = canio.getIfacePerfCounters(i);
        protocol::CANIfaceStats stats;
        stats.errors = can_perf.errors;
        stats.frames_tx = can_perf.frames_tx;
        stats.frames_rx = can_perf.frames_rx;
        resp.can_iface_stats.push_back(stats);
    }
}

int TransportStatsProvider::start()
{
    return srv_.start(GetTransportStatsCallback(this, &TransportStatsProvider::handleGetTransportStats));
}

}

#endif
