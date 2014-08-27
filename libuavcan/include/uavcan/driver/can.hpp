/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/stdint.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/driver/system_clock.hpp>

namespace uavcan
{
/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
UAVCAN_PACKED_BEGIN
struct UAVCAN_EXPORT CanFrame
{
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

    static const uint8_t MaxDataLen = 8;

    uint32_t id;                ///< CAN ID with flags (above)
    uint8_t data[MaxDataLen];
    uint8_t dlc;                ///< Data Length Code

    CanFrame()
        : id(0)
        , dlc(0)
    {
        fill(data, data + MaxDataLen, uint8_t(0));
    }

    CanFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len)
        : id(can_id)
        , dlc((data_len > MaxDataLen) ? MaxDataLen : data_len)
    {
        UAVCAN_ASSERT(can_data != NULL);
        UAVCAN_ASSERT(data_len == dlc);
        (void)copy(can_data, can_data + dlc, this->data);
    }

    bool operator!=(const CanFrame& rhs) const { return !operator==(rhs); }
    bool operator==(const CanFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && equal(data, data + dlc, rhs.data);
    }

    bool isExtended()                  const { return id & FlagEFF; }
    bool isRemoteTransmissionRequest() const { return id & FlagRTR; }
    bool isErrorFrame()                const { return id & FlagERR; }

#if UAVCAN_TOSTRING
    enum StringRepresentation
    {
        StrTight,   ///< Minimum string length (default)
        StrAligned  ///< Fixed formatting for any frame
    };
    std::string toString(StringRepresentation mode = StrTight) const;
#endif

    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const CanFrame& rhs) const;
    bool priorityLowerThan(const CanFrame& rhs) const { return rhs.priorityHigherThan(*this); }
};
UAVCAN_PACKED_END

/**
 * CAN hardware filter config struct.
 * Masks from @ref CanFrame can be applied to define frame type (EFF, EXT, etc.).
 * @ref ICanIface::configureFilters().
 */
struct UAVCAN_EXPORT CanFilterConfig
{
    uint32_t id;
    uint32_t mask;
};

/**
 * Events to look for during @ref ICanDriver::select() call.
 * Bit position defines iface index, e.g. read = 1 << 2 to read from the third iface.
 */
struct UAVCAN_EXPORT CanSelectMasks
{
    uint8_t read;
    uint8_t write;

    CanSelectMasks()
        : read(0)
        , write(0)
    { }
};

/**
 * Special IO flags.
 */
typedef uint16_t CanIOFlags;
static const CanIOFlags CanIOFlagLoopback = 1; ///< Send the frame back to RX with true TX timestamps

/**
 * Single non-blocking CAN interface.
 */
class UAVCAN_EXPORT ICanIface
{
public:
    virtual ~ICanIface() { }

    /**
     * Non-blocking transmission.
     * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
     * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
     */
    virtual int16_t send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags) = 0;

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
    virtual int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                            CanIOFlags& out_flags) = 0;

    /**
     * Configure the hardware CAN filters. @ref CanFilterConfig.
     * @return 0 = success, negative for error.
     */
    virtual int16_t configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) = 0;

    /**
     * Number of available hardware filters.
     */
    virtual uint16_t getNumFilters() const = 0;

    /**
     * Continuously incrementing counter of hardware errors.
     */
    virtual uint64_t getErrorCount() const = 0;
};

/**
 * Generic CAN driver.
 */
class UAVCAN_EXPORT ICanDriver
{
public:
    virtual ~ICanDriver() { }

    /**
     * Returns an interface by index, or null pointer if the index is out of range.
     */
    virtual ICanIface* getIface(uint8_t iface_index) = 0;

    /**
     * Total number of available CAN interfaces.
     * This value shall not change after initialization.
     */
    virtual uint8_t getNumIfaces() const = 0;

    /**
     * Block until the deadline, or one of the specified interfaces becomes available for read or write.
     * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
     * Bit position in the masks defines interface index.
     * Note that it is allowed to return from this method even if no requested events actually happened, or if
     * there are events that were not requested by the lirary.
     * @param [in,out] inout_masks        Masks indicating which interfaces are needed/available for IO.
     * @param [in]     blocking_deadline  Zero means non-blocking operation.
     * @return Positive number of ready interfaces or negative error code.
     */
    virtual int16_t select(CanSelectMasks& inout_masks, MonotonicTime blocking_deadline) = 0;
};

}
