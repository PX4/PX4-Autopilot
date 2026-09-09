#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

Gps::Gps(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
	_checks.setParams(_check_params);
}

Gps::~Gps()
{
}

void Gps::send(const uint64_t time)
{
	const float dt = static_cast<float>(time - _gps_data.time_us - kGpsDelayUs) * 1e-6f;

	_gps_data.time_us = time - kGpsDelayUs;

	if (fabsf(_gps_pos_rate(0)) > FLT_EPSILON || fabsf(_gps_pos_rate(1)) > FLT_EPSILON) {
		stepHorizontalPositionByMeters(Vector2f(_gps_pos_rate) * dt);
	}

	if (fabsf(_gps_pos_rate(2)) > FLT_EPSILON) {
		stepHeightByMeters(-_gps_pos_rate(2) * dt);
	}


	gnssChecksSample sample{};
	sample.time_us = _gps_data.time_us;
	sample.lat = _gps_data.lat;
	sample.lon = _gps_data.lon;
	sample.alt = _gps_data.alt;
	sample.vel = _gps_data.vel;
	sample.hacc = _gps_data.hacc;
	sample.vacc = _gps_data.vacc;
	sample.sacc = _gps_data.sacc;
	sample.fix_type = _gps_data.fix_type;
	sample.nsats = _gps_data.nsats;
	sample.pdop = _gps_data.pdop;
	sample.spoofed = _gps_data.spoofed;
	sample.jammed = _gps_data.jammed;

	// Use the real sensor-layer checks and the same simulated vehicle state as the EKF.
	const auto &control_status = _ekf->control_status_flags();
	_checks.run(sample, control_status.in_air, control_status.vehicle_at_rest);

	gnssCheckStatus checks{};
	checks.checks_passed = _checks.passed();
	checks.initial_checks_passed = _checks.initialChecksPassed();
	checks.time_last_pass_us = _checks.getLastPassUs();
	checks.time_last_fail_us = _checks.getLastFailUs();
	checks.check_fail_status.value = _checks.getFailStatus().value & _checks.getEnabledChecksFailStatusMask();
	checks.position_drift_rate_horizontal_m_s = _checks.horizontal_position_drift_rate_m_s();
	checks.position_drift_rate_vertical_m_s = _checks.vertical_position_drift_rate_m_s();
	checks.filtered_horizontal_speed_m_s = _checks.filtered_horizontal_velocity_m_s();

	// Status topic contract: the EKF applies the latest result to whatever sample it fuses next
	_ekf->setGpsChecksData(checks);
	_ekf->setGpsData(_gps_data);
}

void Gps::setMinRequiredGpsHealthTime(const uint64_t time_us)
{
	_check_params.min_health_time_us = time_us;
	_checks.setParams(_check_params);
}

void Gps::setData(const gnssSample &gps)
{
	_gps_data = gps;
}

void Gps::setAltitude(const float alt)
{
	_gps_data.alt = alt;
}

void Gps::setLatitude(const double lat)
{
	_gps_data.lat = lat;
}

void Gps::setLongitude(const double lon)
{
	_gps_data.lon = lon;
}

void Gps::setVelocity(const Vector3f &vel)
{
	_gps_data.vel = vel;
}

void Gps::setYaw(const float yaw)
{
	_gps_data.yaw = yaw;
}

void Gps::setYawOffset(const float yaw_offset)
{
	_gps_data.yaw_offset = yaw_offset;
}

void Gps::setFixType(const int fix_type)
{
	_gps_data.fix_type = fix_type;
}

void Gps::setNumberOfSatellites(const int num_satellites)
{
	_gps_data.nsats = num_satellites;
}

void Gps::setPdop(const float pdop)
{
	_gps_data.pdop = pdop;
}

void Gps::setPositionRateNED(const Vector3f &rate)
{
	_gps_pos_rate = rate;
}

void Gps::stepHeightByMeters(const float hgt_change)
{
	_gps_data.alt += hgt_change;
}

void Gps::stepHorizontalPositionByMeters(const Vector2f hpos_change)
{
	float hposN_curr {0.f};
	float hposE_curr {0.f};

	double lat_new {0.0};
	double lon_new {0.0};

	_ekf->global_origin().project(_gps_data.lat, _gps_data.lon, hposN_curr, hposE_curr);

	Vector2f hpos_new = Vector2f{hposN_curr, hposE_curr} + hpos_change;

	_ekf->global_origin().reproject(hpos_new(0), hpos_new(1), lat_new, lon_new);

	_gps_data.lon = lon_new;
	_gps_data.lat = lat_new;
}

gnssSample Gps::getDefaultGpsData()
{
	gnssSample gps_data{};
	gps_data.time_us = 0;
	gps_data.lat = 47.3566094;
	gps_data.lon = 8.5190237;
	gps_data.alt = 422.056f;
	gps_data.yaw = NAN;
	gps_data.yaw_offset = 0.0f;
	gps_data.fix_type = 3;
	gps_data.hacc = 0.5f;
	gps_data.vacc = 0.8f;
	gps_data.sacc = 0.2f;
	gps_data.vel.setZero();
	gps_data.nsats = 16;
	gps_data.pdop = 0.0f;

	return gps_data;
}

} // namespace sensor
} // namespace sensor_simulator
