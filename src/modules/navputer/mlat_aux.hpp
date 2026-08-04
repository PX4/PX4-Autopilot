#ifndef MLAT_AUX_HPP
#define MLAT_AUX_HPP

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/ranging_beacon.h>

class MlatAux final
{
public:
	MlatAux() = default;

	void update();

	static constexpr int kMaxBeacons = 8;

private:
	struct BeaconEntry {
		bool valid{false};
		uint8_t beacon_id{0};
		double lat{0.0};
		double lon{0.0};
		float alt{0.0f};
		float range{0.0f};
		float range_accuracy{0.0f};
		uint64_t timestamp_sample{0};
	};

	void updateBeaconStore();
	bool solve();

	uORB::Subscription _ranging_beacon_sub {ORB_ID(ranging_beacon)};
	uORB::Subscription _local_position_sub {ORB_ID(navput_local_position)};
	uORB::PublicationMulti<aux_global_position_s> _aux_global_position_pub {ORB_ID(aux_global_position)};

	BeaconEntry _beacons[kMaxBeacons] {};

	MapProjection _projection;

	hrt_abstime _last_run{0};
};

#endif // MLAT_AUX_HPP
