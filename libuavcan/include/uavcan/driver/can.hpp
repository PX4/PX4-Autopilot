/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <algorithm>
#include <string>
#include <uavcan/stdint.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/driver/system_clock.hpp>

namespace uavcan
{
/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
UAVCAN_PACKED_BEGIN
struct CanFrame
{
    static const uint32_t MaskStdID = 0x000007FF;
    static const uint32_t MaskExtID = 0x1FFFFFFF;
    static const uint32_t FlagEFF = 1 << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1 << 30;                  ///< Remote transmission request

    uint32_t id;        ///< CAN ID with flags (above)
    uint8_t data[8];
    uint8_t dlc;        ///< Data Length Code

    CanFrame()
    : id(0)
    , dlc(0)
    {
        std::fill(data, data + sizeof(data), 0);
    }

    CanFrame(uint32_t id, const uint8_t* data, unsigned int dlc)
    : id(id)
    , dlc(dlc)
    {
        assert(data && dlc <= 8);
        std::copy(data, data + dlc, this->data);
    }

    bool operator!=(const CanFrame& rhs) const { return !operator==(rhs); }
    bool operator==(const CanFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && std::equal(data, data + dlc, rhs.data);
    }

    bool isExtended() const { return id & FlagEFF; }
    bool isRemoteTransmissionRequest() const { return id & FlagRTR; }

    enum StringRepresentation { StrTight, StrAligned };
    std::string toString(StringRepresentation mode = StrTight) const;

    // TODO: priority comparison for EXT vs STD frames
    bool priorityHigherThan(const CanFrame& rhs) const
    {
        return (id & CanFrame::MaskExtID) < (rhs.id & CanFrame::MaskExtID);
    }

    bool priorityLowerThan(const CanFrame& rhs) const
    {
        return (id & CanFrame::MaskExtID) > (rhs.id & CanFrame::MaskExtID);
    }
};
UAVCAN_PACKED_END

/**
 * CAN hardware filter config struct. @ref ICanDriver::filter().
 */
struct CanFilterConfig
{
    uint32_t id;
    uint32_t mask;
};

/**
 * Single non-blocking CAN interface.
 */
class ICanIface
{
public:
    virtual ~ICanIface() { }

    /**
     * Non-blocking transmission.
     * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
     * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
     */
    virtual int send(const CanFrame& frame, MonotonicTime tx_deadline) = 0;

    /**
     * Non-blocking reception.
     * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
     * Monotonic timestamp is required and can be not precise since it is needed only for
     * protocol timing validation (transfer timeouts and inter-transfer intervals).
     * UTC timestamp is optional, if available it will be used for precise time synchronization;
     * must be set to zero if not available.
     * Refer to @ref ISystemClock to learn more about timestamps.
     * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
     * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
     * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
     */
    virtual int receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc) = 0;

    /**
     * Configure the hardware CAN filters. @ref CanFilterConfig.
     * @return 0 = success, negative for error.
     */
    virtual int configureFilters(const CanFilterConfig* filter_configs, int num_configs) = 0;

    /**
     * Number of available hardware filters.
     */
    virtual int getNumFilters() const = 0;

    /**
     * Continuously incrementing counter of detected hardware errors.
     */
    virtual uint64_t getNumErrors() const = 0;
};

/**
 * Generic CAN driver.
 */
class ICanDriver
{
public:
    virtual ~ICanDriver() { }

    /**
     * Returns the interface by index, or null pointer if the index is out of range.
     */
    virtual ICanIface* getIface(int iface_index) = 0;

    /**
     * Total number of available CAN interfaces.
     */
    virtual int getNumIfaces() const = 0;

    /**
     * Block until the deadline, or one of the specified interfaces becomes available for read or write.
     * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
     * Bit position in the masks defines interface index.
     * @param [in,out] inout_write_iface_mask Mask indicating which interfaces are needed/available to write.
     * @param [in,out] inout_read_iface_mask  Same as above for reading.
     * @param [in]     blocking_deadline      Zero means non-blocking operation.
     * @return Positive number of ready interfaces or negative error code.
     */
    virtual int select(int& inout_write_iface_mask, int& inout_read_iface_mask, MonotonicTime blocking_deadline) = 0;
};

}
