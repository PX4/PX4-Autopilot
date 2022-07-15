/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_nuttx/socketcan.hpp>
#include <uavcan_nuttx/clock.hpp>
#include <uavcan/util/templates.hpp>

#define UAVCAN_SOCKETCAN_RX_QUEUE_LEN 64

struct CriticalSectionLocker {
	const irqstate_t flags_;

	CriticalSectionLocker()
		: flags_(enter_critical_section())
	{
	}

	~CriticalSectionLocker()
	{
		leave_critical_section(flags_);
	}
};

namespace uavcan_socketcan
{
namespace
{
/**
 * Hardware message objects are allocated as follows:
 *  - 1 - Single TX object
 *  - 2..32 - RX objects
 * TX priority is defined by the message object number, not by the CAN ID (chapter 16.7.3.5 of the user manual),
 * hence we can't use more than one object because that would cause priority inversion on long transfers.
 */
constexpr unsigned NumberOfMessageObjects   = 32;
constexpr unsigned NumberOfTxMessageObjects = 1;
constexpr unsigned NumberOfRxMessageObjects = NumberOfMessageObjects - NumberOfTxMessageObjects;
constexpr unsigned TxMessageObjectNumber    = 1;

/**
 * Total number of CAN errors.
 * Does not overflow.
 */
volatile std::uint32_t error_cnt = 0;

/**
 * False if there's no pending TX frame, i.e. write is possible.
 */
volatile bool tx_pending = false;

/**
 * Currently pending frame must be aborted on first error.
 */
volatile bool tx_abort_on_error = false;

/**
 * Gets updated every time the CAN IRQ handler is being called.
 */
volatile std::uint64_t last_irq_utc_timestamp = 0;

/**
 * Set by the driver on every successful TX or RX; reset by the application.
 */
volatile bool had_activity = false;

/**
 * After a received message gets extracted from C_CAN, it will be stored in the RX queue until libuavcan
 * reads it via select()/receive() calls.
 */
class RxQueue
{
	struct Item {
		std::uint64_t utc_usec = 0;
		uavcan::CanFrame frame;
	};

	Item buf_[UAVCAN_SOCKETCAN_RX_QUEUE_LEN];
	std::uint32_t overflow_cnt_ = 0;
	std::uint8_t in_ = 0;
	std::uint8_t out_ = 0;
	std::uint8_t len_ = 0;

public:
	void push(const uavcan::CanFrame &frame, const volatile std::uint64_t &utc_usec)
	{
		buf_[in_].frame    = frame;
		buf_[in_].utc_usec = utc_usec;
		in_++;

		if (in_ >= UAVCAN_SOCKETCAN_RX_QUEUE_LEN) {
			in_ = 0;
		}

		len_++;

		if (len_ > UAVCAN_SOCKETCAN_RX_QUEUE_LEN) {
			len_ = UAVCAN_SOCKETCAN_RX_QUEUE_LEN;

			if (overflow_cnt_ < 0xFFFFFFFF) {
				overflow_cnt_++;
			}

			out_++;

			if (out_ >= UAVCAN_SOCKETCAN_RX_QUEUE_LEN) {
				out_ = 0;
			}
		}
	}

	void pop(uavcan::CanFrame &out_frame, std::uint64_t &out_utc_usec)
	{
		if (len_ > 0) {
			out_frame    = buf_[out_].frame;
			out_utc_usec = buf_[out_].utc_usec;
			out_++;

			if (out_ >= UAVCAN_SOCKETCAN_RX_QUEUE_LEN) {
				out_ = 0;
			}

			len_--;
		}
	}

	unsigned getLength() const { return len_; }

	std::uint32_t getOverflowCount() const { return overflow_cnt_; }
};

RxQueue rx_queue;


struct BitTimingSettings {
	std::uint32_t canclkdiv;
	std::uint32_t canbtr;

	bool isValid() const { return canbtr != 0; }
};

} // namespace

CanDriver CanDriver::self;

uavcan::uint32_t CanDriver::detectBitRate(void (*idle_callback)())
{
	//FIXME
	return 1;
}

int CanDriver::init(uavcan::uint32_t bitrate)
{
	{
		//FIXME
	}

	/*
	 * Applying default filter configuration (accept all)
	 */
	if (configureFilters(nullptr, 0) < 0) {
		return -1;
	}

	return 0;
}

bool CanDriver::hasReadyRx() const
{
	CriticalSectionLocker locker;
	return rx_queue.getLength() > 0;
}

bool CanDriver::hasEmptyTx() const
{
	CriticalSectionLocker locker;
	return !tx_pending;
}

bool CanDriver::hadActivity()
{
	CriticalSectionLocker locker;
	const bool ret = had_activity;
	had_activity = false;
	return ret;
}

uavcan::uint32_t CanDriver::getRxQueueOverflowCount() const
{
	CriticalSectionLocker locker;
	return rx_queue.getOverflowCount();
}

bool CanDriver::isInBusOffState() const
{
	//FIXME
	return false;
}

uavcan::int16_t CanDriver::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
				uavcan::CanIOFlags flags)
{
	//FIXME
	return 1;
}

uavcan::int16_t CanDriver::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				   uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	out_ts_monotonic = clock.getMonotonic();
	out_flags = 0;                                            // We don't support any IO flags

	CriticalSectionLocker locker;

	if (rx_queue.getLength() == 0) {
		return 0;
	}

	std::uint64_t ts_utc = 0;
	rx_queue.pop(out_frame, ts_utc);
	out_ts_utc = uavcan::UtcTime::fromUSec(ts_utc);
	return 1;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
				  uavcan::MonotonicTime blocking_deadline)
{


	const bool noblock = ((inout_masks.read  == 1) && hasReadyRx()) ||
			     ((inout_masks.write == 1) && hasEmptyTx());

	if (!noblock && (clock.getMonotonic() > blocking_deadline)) {

	}

	inout_masks.read  = hasReadyRx() ? 1 : 0;

	inout_masks.write = (hasEmptyTx()) ? 1 : 0;     // Disable write while in bus-off

	return 0;           // Return value doesn't matter as long as it is non-negative
}

uavcan::int16_t CanDriver::configureFilters(const uavcan::CanFilterConfig *filter_configs,
		uavcan::uint16_t num_configs)
{
	CriticalSectionLocker locker;

	//FIXME

	return 0;
}

uavcan::uint64_t CanDriver::getErrorCount() const
{
	CriticalSectionLocker locker;
	return std::uint64_t(error_cnt) + std::uint64_t(rx_queue.getOverflowCount());
}

uavcan::uint16_t CanDriver::getNumFilters() const
{
	return NumberOfRxMessageObjects;
}

uavcan::ICanIface *CanDriver::getIface(uavcan::uint8_t iface_index)
{
	return (iface_index == 0) ? this : nullptr;
}

uavcan::uint8_t CanDriver::getNumIfaces() const
{
	return 1;
}

}
