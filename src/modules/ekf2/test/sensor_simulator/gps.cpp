#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

Gps::Gps(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

Gps::~Gps()
{
}

void Gps::send(const uint64_t time)
{
	const float dt = static_cast<float>(time - _gps_data.time_usec) * 1e-6f;

	_gps_data.time_usec = time;

	if (fabsf(_gps_pos_rate(0)) > FLT_EPSILON || fabsf(_gps_pos_rate(1)) > FLT_EPSILON) {
		stepHorizontalPositionByMeters(Vector2f(_gps_pos_rate) * dt);
	}

	if (fabsf(_gps_pos_rate(2)) > FLT_EPSILON) {
		stepHeightByMeters(-_gps_pos_rate(2) * dt);
	}

	_ekf->setGpsData(_gps_data);
}

void Gps::setData(const gps_message &gps)
{
	_gps_data = gps;
}

void Gps::setAltitude(const int32_t alt)
{
	_gps_data.alt = alt;
}

void Gps::setLatitude(const int32_t lat)
{
	_gps_data.lat = lat;
}

void Gps::setLongitude(const int32_t lon)
{
	_gps_data.lon = lon;
}

void Gps::setVelocity(const Vector3f &vel)
{
	_gps_data.vel_ned = vel;
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
	_gps_data.alt += hgt_change * 1e3f;
}

void Gps::stepHorizontalPositionByMeters(const Vector2f hpos_change)
{
	float hposN_curr {0.f};
	float hposE_curr {0.f};

	double lat_new {0.0};
	double lon_new {0.0};

	map_projection_project(&_ekf->global_origin(), _gps_data.lat * 1e-7, _gps_data.lon * 1e-7, &hposN_curr, &hposE_curr);

	Vector2f hpos_new = Vector2f{hposN_curr, hposE_curr} + hpos_change;

	map_projection_reproject(&_ekf->global_origin(), hpos_new(0), hpos_new(1), &lat_new, &lon_new);

	_gps_data.lon = static_cast<int32_t>(lon_new * 1e7);
	_gps_data.lat = static_cast<int32_t>(lat_new * 1e7);
}

gps_message Gps::getDefaultGpsData()
{
	gps_message gps_data{};
	gps_data.time_usec = 0;
	gps_data.lat = 473566094;
	gps_data.lon = 85190237;
	gps_data.alt = 422056;
	gps_data.yaw = NAN;
	gps_data.yaw_offset = 0.0f;
	gps_data.fix_type = 3;
	gps_data.eph = 0.5f;
	gps_data.epv = 0.8f;
	gps_data.sacc = 0.2f;
	gps_data.vel_m_s = 0.0;
	gps_data.vel_ned.setZero();
	gps_data.vel_ned_valid = 1;
	gps_data.nsats = 16;
	gps_data.pdop = 0.0f;

	return gps_data;
}

} // namespace sensor
} // namespace sensor_simulator
