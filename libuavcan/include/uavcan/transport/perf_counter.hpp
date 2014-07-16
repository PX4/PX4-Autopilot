/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/stdint.hpp>
#include <uavcan/build_config.hpp>

namespace uavcan
{

#if UAVCAN_TINY

class UAVCAN_EXPORT TransferPerfCounter
{
public:
    void addTxTransfer() { }
    void addRxTransfer() { }
    void addError() { }
    void addErrors(unsigned) { }
    uint64_t getTxTransferCount() const { return 0; }
    uint64_t getRxTransferCount() const { return 0; }
    uint64_t getErrorCount() const { return 0; }
};

#else

class UAVCAN_EXPORT TransferPerfCounter
{
    uint64_t transfers_tx_;
    uint64_t transfers_rx_;
    uint64_t errors_;

public:
    TransferPerfCounter()
        : transfers_tx_(0)
        , transfers_rx_(0)
        , errors_(0)
    { }

    void addTxTransfer() { transfers_tx_++; }
    void addRxTransfer() { transfers_rx_++; }

    void addError() { errors_++; }

    void addErrors(unsigned errors)
    {
        errors_ += errors;
    }

    uint64_t getTxTransferCount() const { return transfers_tx_; }
    uint64_t getRxTransferCount() const { return transfers_rx_; }
    uint64_t getErrorCount() const { return errors_; }
};

#endif

}
