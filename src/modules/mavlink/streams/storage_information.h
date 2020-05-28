#ifndef MAVLINK_STREAM_STORAGE_INFORMATION_H
#define MAVLINK_STREAM_STORAGE_INFORMATION_H

#include "../mavlink_messages.h"

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#include <math.h>

class MavlinkStreamStorageInformation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamStorageInformation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "STORAGE_INFORMATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STORAGE_INFORMATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStorageInformation(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool request_message(float param2, float param3, float param4,
			     float param5, float param6, float param7) override
	{
		storage_id = (int)roundf(param2);
		return send(hrt_absolute_time());
	}
private:
	int storage_id = 0;

	/* do not allow top copying this class */
	MavlinkStreamStorageInformation(MavlinkStreamStorageInformation &) = delete;
	MavlinkStreamStorageInformation &operator = (const MavlinkStreamStorageInformation &) = delete;


protected:
	explicit MavlinkStreamStorageInformation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_storage_information_t storage_info{};
		const char *microsd_dir = PX4_STORAGEDIR;

		if (storage_id == 0 || storage_id == 1) { // request is for all or the first storage
			storage_info.storage_id = 1;

			struct statfs statfs_buf;
			uint64_t total_bytes = 0;
			uint64_t avail_bytes = 0;

			if (statfs(microsd_dir, &statfs_buf) == 0) {
				total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
				avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
			}

			if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
				storage_info.storage_count = 0;
				storage_info.status = 0; // not available

			} else {
				storage_info.storage_count = 1;
				storage_info.status = 2; // available & formatted
				storage_info.total_capacity = total_bytes / 1024. / 1024.;
				storage_info.available_capacity = avail_bytes / 1024. / 1024.;
				storage_info.used_capacity = (total_bytes - avail_bytes) / 1024. / 1024.;
			}

		} else {
			return false; // results in MAV_RESULT_DENIED
		}

		storage_info.time_boot_ms = t / 1000;
		mavlink_msg_storage_information_send_struct(_mavlink->get_channel(), &storage_info);
		return true;
	}
};

#endif
