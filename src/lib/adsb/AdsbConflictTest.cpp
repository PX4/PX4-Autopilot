#include <gtest/gtest.h>

#include "AdsbConflict.h"

#include "AdsbConflictTest.h"

class TestAdsbConflict : public AdsbConflict
{
public:
	TestAdsbConflict() : AdsbConflict() {}
	~TestAdsbConflict() = default;

	void set_traffic_buffer(const traffic_buffer_s &traffic_buffer)
	{
		_traffic_buffer = traffic_buffer;
	}

	void set_conflict(bool &conflict_detected)
	{
		_conflict_detected = conflict_detected;
	}
};


TEST_F(AdsbConflictTest, detectTrafficConflict)
{
	int collision_time_threshold = 60;

	float crosstrack_separation = 500.0f;
	float vertical_separation = 500.0f;

	double lat_now = 32.617013;
	double lon_now = -96.490564;
	float alt_now = 1000.0f;

	float vx_now = 0.0f;
	float vy_now = 0.0f;
	float vz_now = 0.0f;

	uint32_t traffic_dataset_size = sizeof(traffic_dataset) / sizeof(traffic_dataset[0]);

	TestAdsbConflict 	adsb_conflict;

	adsb_conflict.set_conflict_detection_params(crosstrack_separation, vertical_separation, collision_time_threshold, 1);

	for (uint32_t i = 0; i < traffic_dataset_size; i++) {

		struct traffic_data_s traffic = traffic_dataset[i];

		// GIVEN traffic dataset (which should result in conflict)
		adsb_conflict._transponder_report.lat = traffic.lat_traffic;
		adsb_conflict._transponder_report.lon = traffic.lon_traffic;
		adsb_conflict._transponder_report.altitude = traffic.alt_traffic;
		adsb_conflict._transponder_report.heading = traffic.heading_traffic;
		adsb_conflict._transponder_report.hor_velocity = traffic.vxy_traffic;
		adsb_conflict._transponder_report.ver_velocity = traffic.vz_traffic;

		// WHEN detect traffic conflict is called
		adsb_conflict.detect_traffic_conflict(lat_now, lon_now, alt_now, vx_now, vy_now, vz_now);

		// THEN expect conflict to be detected
		EXPECT_TRUE(adsb_conflict._conflict_detected == traffic.in_conflict);
	}
}


TEST_F(AdsbConflictTest, trafficAlerts)
{
	struct	traffic_buffer_s used_buffer;
	used_buffer.icao_address.push_back(2345);
	used_buffer.icao_address.push_back(1234);
	used_buffer.icao_address.push_back(1897);
	used_buffer.icao_address.push_back(0567);
	used_buffer.icao_address.push_back(8685);
	used_buffer.icao_address.push_back(5000);

	used_buffer.timestamp.push_back(3_s);
	used_buffer.timestamp.push_back(800_s);
	used_buffer.timestamp.push_back(100_s);
	used_buffer.timestamp.push_back(20000_s);
	used_buffer.timestamp.push_back(6000_s);
	used_buffer.timestamp.push_back(6587_s);

	struct	traffic_buffer_s full_buffer;
	full_buffer.icao_address.push_back(2345);
	full_buffer.icao_address.push_back(1234);
	full_buffer.icao_address.push_back(1897);
	full_buffer.icao_address.push_back(0567);
	full_buffer.icao_address.push_back(8685);
	full_buffer.icao_address.push_back(5000);
	full_buffer.icao_address.push_back(0000);
	full_buffer.icao_address.push_back(2);
	full_buffer.icao_address.push_back(589742397);
	full_buffer.icao_address.push_back(99999);

	full_buffer.timestamp.push_back(1_s);
	full_buffer.timestamp.push_back(800_s);
	full_buffer.timestamp.push_back(100_s);
	full_buffer.timestamp.push_back(20000_s);
	full_buffer.timestamp.push_back(6000_s);
	full_buffer.timestamp.push_back(19000_s);
	full_buffer.timestamp.push_back(5000_s);
	full_buffer.timestamp.push_back(2_s);
	full_buffer.timestamp.push_back(1000_s);
	full_buffer.timestamp.push_back(58943_s);

	struct traffic_buffer_s empty_buffer = {};

	TestAdsbConflict 	adsb_conflict;

	// GIVEN used buffer
	adsb_conflict.set_traffic_buffer(used_buffer);

	// WHEN no conflict detected
	bool conflict_detected  = false;
	hrt_abstime now = 0_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 00001;
	adsb_conflict.get_traffic_state(now);

	// THEN expect no conflict
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::NO_CONFLICT);

	// GIVEN conflict detected
	conflict_detected  = true;
	now = 1_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 9876;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict to be added to buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::ADD_CONFLICT);

	// GIVEN empty buffer
	adsb_conflict.set_traffic_buffer(empty_buffer);

	// WHEN conflict detected
	conflict_detected  = true;
	now = 0_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 9876;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict to be added to buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::ADD_CONFLICT);

	// GIVEN full buffer
	adsb_conflict.set_traffic_buffer(full_buffer);

	// WHEN conflict detected
	conflict_detected  = true;
	now = 1_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 7777;
	adsb_conflict.get_traffic_state(now);

	// THEN expect buffer full
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::BUFFER_FULL);

	// WHEN conflict set to false again
	conflict_detected  = false;
	now = 2_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 7777;
	adsb_conflict.get_traffic_state(now);

	// THEN expect no conflict message
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::NO_CONFLICT);

	// WHEN existing conflict is set to false
	conflict_detected  = false;
	now = 3_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict to be removed from buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::REMOVE_OLD_CONFLICT);

	// GIVEN used buffer
	adsb_conflict.set_traffic_buffer(used_buffer);

	// WHEN conflict is set to false again
	conflict_detected  = false;
	now = 0_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict to be removed from buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::REMOVE_OLD_CONFLICT);
}

TEST_F(AdsbConflictTest, trafficReminder)
{
	struct	traffic_buffer_s used_buffer;
	used_buffer.icao_address.push_back(2345);
	used_buffer.icao_address.push_back(1234);
	used_buffer.icao_address.push_back(1897);
	used_buffer.icao_address.push_back(0567);
	used_buffer.icao_address.push_back(8685);
	used_buffer.icao_address.push_back(5000);

	used_buffer.timestamp.push_back(3_s);
	used_buffer.timestamp.push_back(80_s);
	used_buffer.timestamp.push_back(10_s);
	used_buffer.timestamp.push_back(1000_s);
	used_buffer.timestamp.push_back(100_s);
	used_buffer.timestamp.push_back(187_s);

	struct	traffic_buffer_s full_buffer;
	full_buffer.icao_address.push_back(2345);
	full_buffer.icao_address.push_back(1234);
	full_buffer.icao_address.push_back(1897);
	full_buffer.icao_address.push_back(0567);
	full_buffer.icao_address.push_back(8685);
	full_buffer.icao_address.push_back(5000);
	full_buffer.icao_address.push_back(0000);
	full_buffer.icao_address.push_back(2);
	full_buffer.icao_address.push_back(589742397);
	full_buffer.icao_address.push_back(99999);

	full_buffer.timestamp.push_back(1_s);
	full_buffer.timestamp.push_back(80_s);
	full_buffer.timestamp.push_back(10_s);
	full_buffer.timestamp.push_back(1000_s);
	full_buffer.timestamp.push_back(100_s);
	full_buffer.timestamp.push_back(900_s);
	full_buffer.timestamp.push_back(500_s);
	full_buffer.timestamp.push_back(2_s);
	full_buffer.timestamp.push_back(100_s);
	full_buffer.timestamp.push_back(5843_s);

	TestAdsbConflict 	adsb_conflict;

	// GIVEN buffer with 8685 at t=100
	adsb_conflict.set_traffic_buffer(used_buffer);

	// WHEN conflict detected at t=200
	bool conflict_detected  = true;
	hrt_abstime now = 200_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict reminder
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::REMIND_CONFLICT);

	// WHEN INSTEAD conflict is detected only 1s later
	conflict_detected  = true;
	now = 201_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN do not sent conflict notification again
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::NO_CONFLICT);

	// GIVEN full buffer with 8685 at t=100
	adsb_conflict.set_traffic_buffer(full_buffer);

	// WHEN conflict detected at t=400
	conflict_detected  = true;
	now = 400_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict reminder
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::REMIND_CONFLICT);

	// WHEN INSTEAD conflict is detected only 1s later
	conflict_detected  = true;
	now = 401_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN do not sent conflict notification again
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::NO_CONFLICT);

	// WHEN conflict is set to false
	conflict_detected  = false;
	now = 600_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 8685;
	adsb_conflict.get_traffic_state(now);

	// THEN expect conflict to be removed from buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::REMOVE_OLD_CONFLICT);

	// WHEN new conflict is detected
	conflict_detected  = true;
	now = 700_s;
	adsb_conflict.set_conflict(conflict_detected);
	adsb_conflict._transponder_report.icao_address = 7777;
	adsb_conflict.get_traffic_state(now);

	// THEN expect new conflict to be added to buffer
	printf("adsb_conflict._traffic_state %d \n", adsb_conflict._traffic_state);
	EXPECT_TRUE(adsb_conflict._traffic_state == TRAFFIC_STATE::ADD_CONFLICT);
}
