#ifndef UAVCAN_KDECAN_HPP_INCLUDED
#define UAVCAN_KDECAN_HPP_INCLUDED

#include "third_party_protocols.hpp"

namespace kdecan
{

static const int escNodeIdBroadcast = 1;
static const int escNodeIdOffset = 2;
static const int minPwmValue = 1100;
static const int maxPwmValue = 1940;

static const int MASTER_NODE_ID = 0;

enum kdeCanObjAddr
{
	ESCInformation = 0,
	PWMThrottle = 1,
	ESCInputThrottle = 6,
	ESCOutputThrottle = 7,
	ESCStatus = 11,
	GetMCUId = 8,
	UpdateNodeAddress = 9,
	StartESCEnumeration = 10,
	Shutdown = 32,
	Restart = 33,
	Invalid = 50
};

class KdeFrame
{
public:
	static const uint8_t PayloadCapacity = 8;

	KdeFrame(const uavcan::CanRxFrame& in_can_frame)
	{
		parse(in_can_frame);
	}

	KdeFrame(const uint8_t source_address,
		 const uint8_t destination_address,
		 const kdeCanObjAddr object_address,
		 const uint8_t* data,
		 const uint8_t data_length) :
		source_address_(source_address),
		destination_address_(destination_address),
		object_address_(object_address)
	{
		const uint8_t adjusted_data_length = (PayloadCapacity > data_length) ? data_length : PayloadCapacity;

		memcpy(data_, data, adjusted_data_length);
		data_length_ = adjusted_data_length;
	}

	uint8_t getSourceAddress() const { return source_address_; }
	uint8_t getDestinationAddress() const { return destination_address_; }
	kdeCanObjAddr getObjectAddress() const { return object_address_; }
	const uint8_t *getData() const { return data_; }

	void parse(const uavcan::CanRxFrame& in_can_frame)
	{
		uint32_t can_id = in_can_frame.id;

		uint8_t object_address;

		const bool extended_id = (in_can_frame.id & uavcan::CanFrame::FlagEFF) != 0;

		if (extended_id)
		{
			object_address = can_id & 0x000000FF;
			destination_address_ = (can_id & 0x0000FF00) >> 8;
			source_address_ = (can_id & 0x00FF0000) >> 16;
			data_length_ = in_can_frame.dlc;

			if (destination_address_ != MASTER_NODE_ID)
			{
				object_address = Invalid;
			}
		}
		else
		{
			object_address = can_id & 0x0000001F;
			source_address_ = (can_id >> 5) & 0x0000001F; // ((can_id & 0x000003E0) >> 5);
			destination_address_ = (can_id >> 10) & 0x0000001F; // ((can_id & 0x00000400) >> 10);
			data_length_ = in_can_frame.dlc;

			if (destination_address_ != 1)
			{
				object_address = Invalid;
			}
		}

		switch(object_address)
		{
			case ESCInformation: { object_address_ = (data_length_ >= 5) ? ESCInformation : Invalid; } break;
			case PWMThrottle: { object_address_ = (data_length_ >= 2) ? PWMThrottle : Invalid; } break;
			case ESCInputThrottle: { object_address_ = (data_length_ >= 2) ? ESCInputThrottle : Invalid; } break;
			case ESCOutputThrottle: { object_address_ = (data_length_ >= 1) ? ESCOutputThrottle : Invalid; } break;
			case ESCStatus: { object_address_ = (data_length_ >= 8) ? ESCStatus : Invalid; } break;
			case GetMCUId: { object_address_ = (data_length_ >= 8) ? GetMCUId : Invalid; } break;
			case UpdateNodeAddress: { object_address_ = (data_length_ >= 1) ? UpdateNodeAddress : Invalid; } break;
			case StartESCEnumeration: { object_address_ = (data_length_ >= 8) ? StartESCEnumeration : Invalid; } break;
			case Shutdown: { object_address_ = (data_length_ >= 1) ? Shutdown : Invalid; } break;
			case Restart: { object_address_ = (data_length_ >= 1) ? Restart : Invalid; } break;
			default: { object_address_ = Invalid; }
		}

		memcpy(data_, in_can_frame.data, data_length_);
	}

	bool compile(uavcan::CanFrame& out_can_frame, const bool extended_id)
	{
		return compile(out_can_frame.id, out_can_frame.dlc, out_can_frame.data, extended_id);
	}

	bool compile(uint32_t& id, uint8_t& dlc, uint8_t* data, const bool extended_id)

	{
		if (extended_id)
		{
			id = ((uint32_t)0x00000000) |
			     ((uint32_t)object_address_) |
			     (((uint32_t)destination_address_) << 8) |
			     (((uint32_t)source_address_) << 16) |
			     uavcan::CanFrame::FlagEFF;
		}
		else
		{
			id = ((uint32_t)0x00000000) |
			     ((uint32_t)object_address_ & 0x0000001F) |
			     (((uint32_t)destination_address_ & 0x0000001F) << 5);
		}

		switch(object_address_)
		{
			case ESCInformation: { dlc = 0; } break;
			case PWMThrottle: { dlc = 2; } break;
			case ESCInputThrottle: { dlc = 0; } break;
			case ESCOutputThrottle: { dlc = 0; } break;
			case ESCStatus: { dlc = 0; } break;
			case GetMCUId: { dlc = 0; } break;
			case UpdateNodeAddress: { dlc = 8; } break;
			case StartESCEnumeration: { dlc = 2; } break;
			case Shutdown: { dlc = 0; } break;
			case Restart: { dlc = 0; } break;

			default: { return false; }
		}

		// this function will take care of little/big endian conversions
		(void)uavcan::copy(data_, data_ + dlc, data);

		return true;
	}
private:
	uint8_t source_address_;
	uint8_t destination_address_;
	kdeCanObjAddr object_address_;
	uint8_t data_[PayloadCapacity];
	uint8_t data_length_;
};


class KdecanType
{
public:
	KdecanType() { valid_ = true; }

	virtual bool parse(const KdeFrame& frame) { return true; }

	virtual void generate_data(uint8_t* data) { memset(data, 0, KdeFrame::PayloadCapacity); }

	bool isValid() const { return valid_; }

protected:
	bool valid_;
};


class EscStatus : public KdecanType
{
public:
	static const kdeCanObjAddr object_address_ = ESCStatus;

	uint8_t source_address_;

	float voltage_;
	float current_;
	uint32_t erpm_;
	uint16_t temperature_;
	uint16_t warnings_;

	EscStatus() { valid_ = false;}

	EscStatus(const KdeFrame& frame) { valid_ = parse(frame); }

	EscStatus(uint8_t source_address, float voltage = 0.0f, float current = 0.0f, uint32_t erpm = 0, uint16_t temperature = 0, uint16_t warnings = 0) :
		KdecanType(),
		source_address_(source_address),
		voltage_(voltage),
		current_(current),
		erpm_(erpm),
		temperature_(temperature),
		warnings_(warnings)
	{ ; }

	bool parse(const KdeFrame& frame) override
	{
		source_address_ = frame.getSourceAddress();

		if (frame.getObjectAddress() == object_address_)
		{
			const uint8_t* data = frame.getData();

			voltage_ = (float)(data[1] + (0xFF00 & (data[0] << 8))) / 100.0f;
			current_ = (float)(data[3] + (0xFF00 & (data[2] << 8))) / 100.0f;
			erpm_ = (int)(data[5] + (0xFF00 & (data[4] << 8))) * 60 * 2;
			temperature_ = data[6];
			warnings_ = data[7];

			return true;
		}

		return false;
	}
};

class InputThrottle : public KdecanType
{
public:
	static const kdeCanObjAddr object_address_ = ESCInputThrottle;

	uint8_t source_address_;

	uint16_t input_throttle_;

	InputThrottle() { valid_ = false;}

	InputThrottle(const KdeFrame& frame) { valid_ = parse(frame); }

	InputThrottle(uint8_t source_address, float input_throttle = 0.0f) :
		KdecanType(),
		source_address_(source_address),
		input_throttle_(input_throttle)
	{ ; }

	bool parse(const KdeFrame& frame) override
	{
		source_address_ = frame.getSourceAddress();

		if (frame.getObjectAddress() == object_address_)
		{
			const uint8_t* data = frame.getData();

			input_throttle_ = (data[1] + (0xFF00 & (data[0] << 8)));

			return true;
		}

		return false;
	}
};

class OutputThrottle : public KdecanType
{
public:
	static const kdeCanObjAddr object_address_ = ESCOutputThrottle;

	uint8_t source_address_;

	float output_throttle_;

	OutputThrottle() { valid_ = false; }

	OutputThrottle(const KdeFrame& frame) { valid_ = parse(frame); }

	OutputThrottle(uint8_t source_address, float output_throttle = 0.0f) :
		KdecanType(),
		source_address_(source_address),
		output_throttle_(output_throttle)
	{ ; }

	bool parse(const KdeFrame& frame) override
	{
		source_address_ = frame.getSourceAddress();

		if (frame.getObjectAddress() == object_address_)
		{
			const uint8_t* data = frame.getData();

			output_throttle_ = (float)data[0] / 100.0f;

			return true;
		}

		return false;
	}
};

class PwmThrottle : public KdecanType
{
public:
	static const kdeCanObjAddr object_address_ = PWMThrottle;

	uint8_t source_address_;

	uint16_t pwm_throttle_;

	PwmThrottle() { valid_ = false; }

	PwmThrottle(const KdeFrame& frame) { valid_ = parse(frame); }

	PwmThrottle(uint8_t source_address, uint16_t pwm_throttle) :
		KdecanType(),
		source_address_(source_address),
		pwm_throttle_(pwm_throttle)
	{
		valid_ = true;
	}

	void generate_data(uint8_t* data) override
	{
		data[0] = (uint8_t)((0xFF00 & pwm_throttle_) >> 8);
		data[1] = (uint8_t)(0x00FF & pwm_throttle_);
	}
};



template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
	typename Callback_ = std::function<void (const DataType_&)>
#else
	typename Callback_ = void (*)(const DataType_&)
#endif
>
class Subscriber
{
public:
	typedef Subscriber<DataType_, Callback_> SelfType;
	typedef Callback_ Callback;

	class kdeTransferForwarder : public uavcan::CustomTransferListener
	{
	public:
		SelfType& obj_;

		bool handleFrame(const uavcan::CanRxFrame& can_frame, uavcan::Protocol protocol) override
		{
			if (protocol == kdecan::protocolID || !can_frame.isExtended())
			{
				return obj_.handleIncomingTransfer(can_frame);
			}
			else
			{
				return false;
			}
		}

		kdeTransferForwarder(SelfType& obj) :
			uavcan::CustomTransferListener(kdecan::protocolID),
			obj_(obj)
		{;}
	};

	uavcan::INode& node_;
	Callback callback_;
	kdeTransferForwarder forwarder_;
	kdeCanObjAddr target_object_address_;

	Subscriber(uavcan::INode& node) :
		node_(node),
		callback_(),
		forwarder_(*this),
		target_object_address_(DataType_::object_address_)
	{;}

	bool handleIncomingTransfer(const uavcan::CanRxFrame& can_frame)
	{
		const KdeFrame incoming_frame(can_frame);
		const DataType_ received_structure = DataType_(incoming_frame);

		if (received_structure.isValid())
		{
			if(callback_)
			{
				callback_(received_structure);
			}

			return true;
		}
		else
		{
			return false;
		}
	}

	void setCallback(Callback callback)
	{
		callback_ = callback;
	}

	uavcan::CustomTransferListener* getKdeListener()
	{
		return &(this->forwarder_);
	}
};

template <typename DataType_>
class Publisher
{
public:
	uavcan::INode& node_;
	kdeCanObjAddr target_object_address_;

	Publisher(uavcan::INode& node) :
		node_(node),
		target_object_address_(DataType_::object_address_)
	{;}

	bool publish(DataType_ outgoing_structure, const bool extended_id)
	{
		uint8_t data[KdeFrame::PayloadCapacity];

		outgoing_structure.generate_data(data);

		KdeFrame kde_frame(MASTER_NODE_ID, outgoing_structure.source_address_, target_object_address_, data, KdeFrame::PayloadCapacity);

		uavcan::CanFrame can_frame;
		kde_frame.compile(can_frame, extended_id);

		// in case we are not using the extended ID, we are assuming that we should be using the same protocol as the
		// standard UAVCAN protocol
		node_.getDispatcher().getCanIOManager().send(can_frame,
							     extended_id ? kdecan::protocolID : uavcan::Protocol::Standard,
							     node_.getMonotonicTime() + uavcan::MonotonicDuration::fromMSec(100),
							     uavcan::MonotonicTime(),
							     (uint8_t)0xFF,
							     uavcan::CanTxQueue::Qos::Volatile,
							     uavcan::CanIOFlags(0));

		// so far we do no checks in this
		return true;
	}
};

};

#endif // UAVCAN_KDECAN_HPP_INCLUDED
