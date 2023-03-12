#ifndef ACTUATOR_OUTPUTS_HPP
#define ACTUATOR_OUTPUTS_HPP

#include <uORB/topics/actuator_outputs.h>

class MavlinkStreamActuatorOutputs : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamActuatorOutputs(mavlink); }

	static constexpr const char *get_name_static() { return "ACTUATOR_OUTPUTS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ACTUATOR_OUTPUTS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_actuator_outputs_sub.advertised()) {
			return MAVLINK_MSG_ID_ACTUATOR_OUTPUTS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamActuatorOutputs(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};

	bool send() override
	{
		if (_actuator_outputs_sub.updated()) {

			actuator_outputs_s actuator_outputs{};
			_actuator_outputs_sub.copy(&actuator_outputs);

			mavlink_actuator_outputs_t msg{};

			msg.noutputs = actuator_outputs.noutputs;			// valid outputs
			msg.output[0] = actuator_outputs.output[0];			// output data, in natural output units
			msg.output[1] = actuator_outputs.output[1];			// output data, in natural output units
			msg.output[2] = actuator_outputs.output[2];			// output data, in natural output units
			msg.output[3] = actuator_outputs.output[3];			// output data, in natural output units
			msg.output[4] = actuator_outputs.output[4];			// output data, in natural output units
			msg.output[5] = actuator_outputs.output[5];			// output data, in natural output units
			msg.output[6] = actuator_outputs.output[6];			// output data, in natural output units
			msg.output[7] = actuator_outputs.output[7];			// output data, in natural output units
			msg.output[8] = actuator_outputs.output[8];			// output data, in natural output units
			msg.output[9] = actuator_outputs.output[9];			// output data, in natural output units
			msg.output[10] = actuator_outputs.output[10];			// output data, in natural output units
			msg.output[11] = actuator_outputs.output[11];			// output data, in natural output units
			msg.output[12] = actuator_outputs.output[12];			// output data, in natural output units
			msg.output[13] = actuator_outputs.output[13];			// output data, in natural output units
			msg.output[14] = actuator_outputs.output[14];			// output data, in natural output units
			msg.output[15] = actuator_outputs.output[15];			// output data, in natural output units

			mavlink_msg_actuator_outputs_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // ACTUATOR_OUTPUTS_HPP
