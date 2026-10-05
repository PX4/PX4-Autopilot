#include "gnss.h"

namespace sensor_simulator
{
namespace sensor
{

Gnss::Gnss(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
	// The defaults the EKF used before the checks moved to the sensors module
	_check_params.check_mask = 1045;
	_check_params.req_nsats = 6;
	_check_params.req_pdop = 2.f;
	_check_params.req_eph = 5.f;
	_check_params.req_epv = 8.f;
	_check_params.req_sacc = 1.f;
	_check_params.req_hdrift = 0.3f;
	_check_params.req_vdrift = 0.5f;
	_check_params.req_fix = 3;
	_check_params.min_health_time_us = 10'000'000;
	_checks.setParams(_check_params);
}

Gnss::~Gnss()
{
}

void Gnss::send(const uint64_t time)
{
	const float dt = static_cast<float>(time - _gnss_data.time_us - kGnssDelayUs) * 1e-6f;

	_gnss_data.time_us = time - kGnssDelayUs;

	if (fabsf(_gnss_pos_rate(0)) > FLT_EPSILON || fabsf(_gnss_pos_rate(1)) > FLT_EPSILON) {
		stepHorizontalPositionByMeters(Vector2f(_gnss_pos_rate) * dt);
	}

	if (fabsf(_gnss_pos_rate(2)) > FLT_EPSILON) {
		stepHeightByMeters(-_gnss_pos_rate(2) * dt);
	}

	gnssChecksSample sample{};
	sample.time_us = _gnss_data.time_us;
	sample.lat = _gnss_data.lat;
	sample.lon = _gnss_data.lon;
	sample.alt = _gnss_data.alt;
	sample.vel = _gnss_data.vel;
	sample.hacc = _gnss_data.hacc;
	sample.vacc = _gnss_data.vacc;
	sample.sacc = _gnss_data.sacc;
	sample.fix_type = _gnss_data.fix_type;
	sample.nsats = _gnss_data.nsats;
	sample.pdop = _gnss_data.pdop;
	sample.spoofed = _gnss_data.spoofed;
	sample.jammed = _gnss_data.jammed;

	const auto &control_status = _ekf->control_status_flags();
	_gnss_data.usable = _checks.run(sample, control_status.armed, control_status.in_air, control_status.vehicle_at_rest);

	_ekf->setGnssData(_gnss_data);
}

void Gnss::setMinRequiredGnssHealthTime(uint64_t time_us)
{
	_check_params.min_health_time_us = time_us;
	_checks.setParams(_check_params);
	_ekf->set_min_required_gnss_health_time(time_us);
}

void Gnss::setCheckMask(int32_t check_mask)
{
	_check_params.check_mask = check_mask;
	_checks.setParams(_check_params);
}

void Gnss::setData(const gnssSample &gps)
{
	_gnss_data = gps;
}

void Gnss::setAltitude(const float alt)
{
	_gnss_data.alt = alt;
}

void Gnss::setLatitude(const double lat)
{
	_gnss_data.lat = lat;
}

void Gnss::setLongitude(const double lon)
{
	_gnss_data.lon = lon;
}

void Gnss::setVelocity(const Vector3f &vel)
{
	_gnss_data.vel = vel;
}

void Gnss::setFixType(const int fix_type)
{
	_gnss_data.fix_type = fix_type;
}

void Gnss::setNumberOfSatellites(const int num_satellites)
{
	_gnss_data.nsats = num_satellites;
}

void Gnss::setPdop(const float pdop)
{
	_gnss_data.pdop = pdop;
}

void Gnss::setPositionRateNED(const Vector3f &rate)
{
	_gnss_pos_rate = rate;
}

void Gnss::stepHeightByMeters(const float hgt_change)
{
	_gnss_data.alt += hgt_change;
}

void Gnss::stepHorizontalPositionByMeters(const Vector2f hpos_change)
{
	float hposN_curr {0.f};
	float hposE_curr {0.f};

	double lat_new {0.0};
	double lon_new {0.0};

	_ekf->global_origin().project(_gnss_data.lat, _gnss_data.lon, hposN_curr, hposE_curr);

	Vector2f hpos_new = Vector2f{hposN_curr, hposE_curr} + hpos_change;

	_ekf->global_origin().reproject(hpos_new(0), hpos_new(1), lat_new, lon_new);

	_gnss_data.lon = lon_new;
	_gnss_data.lat = lat_new;
}

gnssSample Gnss::getDefaultGnssData()
{
	gnssSample gnss_data{};
	gnss_data.time_us = 0;
	gnss_data.lat = 47.3566094;
	gnss_data.lon = 8.5190237;
	gnss_data.alt = 422.056f;
	gnss_data.fix_type = 3;
	gnss_data.hacc = 0.5f;
	gnss_data.vacc = 0.8f;
	gnss_data.sacc = 0.2f;
	gnss_data.vel.setZero();
	gnss_data.nsats = 16;
	gnss_data.pdop = 0.0f;

	return gnss_data;
}

} // namespace sensor
} // namespace sensor_simulator
