/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#else
# error "Unknown OS"
#endif

#include <uavcan/driver/can.hpp>

namespace uavcan_stm32
{

enum { CanFiltersPerIface = 14 };


class CanIface : public uavcan::ICanIface
{
    CAN_TypeDef* can_;

    uavcan::uint64_t error_cnt_;

public:
    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags);

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags);

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                             uavcan::uint16_t num_configs);

    virtual uavcan::uint16_t getNumFilters() const { return CanFiltersPerIface; }

    virtual uavcan::uint64_t getErrorCount() const { return error_cnt_; }
};

}
