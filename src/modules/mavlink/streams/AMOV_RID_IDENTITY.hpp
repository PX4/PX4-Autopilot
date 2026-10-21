/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#ifndef AMOV_RID_IDENTITY_HPP
#define AMOV_RID_IDENTITY_HPP
#include <uORB/topics/rid_identity.h>
class MavlinkStreamAmovRidIdentity : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAmovRidIdentity(mavlink); }
	static constexpr const char *get_name_static() { return "AMOV_RID_IDENTITY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AMOV_RID_IDENTITY; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override { return MAVLINK_MSG_ID_AMOV_RID_IDENTITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }
private:
	explicit MavlinkStreamAmovRidIdentity(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	uORB::Subscription _sub{ORB_ID(rid_identity)};
	bool send() override
	{
		rid_identity_s id{};

		if (!_sub.update(&id)) { return false; }

		mavlink_amov_rid_identity_t out{};
		out.boot_id = id.boot_id;
		out.session_id = id.session_id;
		out.request_id = id.request_id;
		out.identity_revision = id.identity_revision;
		out.rid_boot_id = id.rid_boot_id;
		out.pending_revision = id.pending_revision;
		out.target_system = id.target_system;
		out.target_component = id.target_component;
		out.protocol_version = 1;
		out.result = id.result;
		out.activation_state = id.activation_state;
		out.rid_node_id = id.rid_node_id;
		out.aircraft_class = id.aircraft_class;
		out.activation_error = id.activation_error;
		memcpy(out.rid_uid, id.rid_uid, sizeof(out.rid_uid));
		memcpy(out.uas_id, id.uas_id, sizeof(out.uas_id));
		memcpy(out.registration_id, id.registration_id, sizeof(out.registration_id));
		memcpy(out.uin, id.uin, sizeof(out.uin));
		memcpy(out.identity_hash, id.identity_hash, sizeof(out.identity_hash));
		memcpy(out.error, id.error, sizeof(out.error));
		mavlink_msg_amov_rid_identity_send_struct(_mavlink->get_channel(), &out);
		return true;
	}
};
#endif
