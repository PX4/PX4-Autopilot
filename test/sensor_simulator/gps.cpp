#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

Gps::Gps(std::shared_ptr<Ekf> ekf):Sensor(ekf)
{
}

Gps::~Gps()
{
}

void Gps::send(uint64_t time)
{
	_gps_data.time_usec = time;
	_ekf->setGpsData(_gps_data);
}

void Gps::setData(const gps_message& gps)
{
	_gps_data = gps;
}

void Gps::setAltitude(int32_t alt)
{
	_gps_data.alt = alt;
}

void Gps::setLatitude(int32_t lat)
{
	_gps_data.lat = lat;
}

void Gps::setLongitude(int32_t lon)
{
	_gps_data.lon = lon;
}

void Gps::setVelocity(const Vector3f& vel)
{
	_gps_data.vel_ned = vel;
}

void Gps::setFixType(int n)
{
	_gps_data.fix_type = n;
}

void Gps::setNumberOfSatellites(int n)
{
	_gps_data.nsats = n;
}

void Gps::setPdop(float pdop)
{
	_gps_data.pdop = pdop;
}

void Gps::stepHeightByMeters(float hgt_change)
{
	_gps_data.alt += hgt_change * 1e3f;
}

void Gps::stepHorizontalPositionByMeters(Vector2f hpos_change)
{
	float hposN_curr;
	float hposE_curr;
	map_projection_global_project((float)_gps_data.lat, (float)_gps_data.lon, &hposN_curr, &hposE_curr);
	Vector2f hpos_new = Vector2f{hposN_curr, hposE_curr} + hpos_change;
	double lat_new;
	double lon_new;
	map_projection_global_reproject(hpos_new(0), hpos_new(1), &lat_new, &lon_new);
	_gps_data.lon = (uint32_t)lon_new;
	_gps_data.lat = (uint32_t)lat_new;
}


gps_message Gps::getDefaultGpsData()
{
	gps_message gps_data{};
	gps_data.time_usec = 0;
	gps_data.lat = 473566094;
	gps_data.lon = 85190237;
	gps_data.alt = 422056;
	gps_data.yaw = 0.0f;
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
