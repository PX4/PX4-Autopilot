#include "SensorSimulator.h"

namespace simulator
{

SensorSimulator::SensorSimulator(Ekf* ekf)
{
	_ekf = ekf;
	setGpsMessageToDefaul();
}

SensorSimulator::~SensorSimulator()
{

}

void SensorSimulator::setGpsMessageToDefaul()
{
	// setup gps message to reasonable default values
	_gps_message.time_usec = 0;
	_gps_message.lat = 473566094;
	_gps_message.lon = 85190237;
	_gps_message.alt = 422056;
	_gps_message.yaw = 0.0f;
	_gps_message.yaw_offset = 0.0f;
	_gps_message.fix_type = 3;
	_gps_message.eph = 0.5f;
	_gps_message.epv = 0.8f;
	_gps_message.sacc = 0.2f;
	_gps_message.vel_m_s = 0.0;
	_gps_message.vel_ned[0] = 0.0f;
	_gps_message.vel_ned[1] = 0.0f;
	_gps_message.vel_ned[2] = 0.0f;
	_gps_message.vel_ned_valid = 1;
	_gps_message.nsats = 16;
	_gps_message.pdop = 0.0f;
}

void SensorSimulator::update_with_const_sensors(uint32_t duration_us,
		Vector3f ang_vel, Vector3f accel,
		Vector3f mag_data, float baro_data)
{
	// store start time
	uint32_t start_time_us = _t_us;

	// compute update time step such that we can update the basic sensor at different rates
	_update_dt_us = gcd(_imu_dt_us, gcd(_mag_dt_us, gcd(_baro_dt_us,_gps_dt_us)));

	// update EKF with synthetic sensor measurements
	for( ; _t_us < start_time_us+duration_us; _t_us += _update_dt_us)
	{
		// Check which sensors update we should do
		if(_fuse_imu && !(_t_us %_imu_dt_us))
		{
			// push imu data into estimator
			imuSample imu_sample_new;
			imu_sample_new.time_us = _t_us;
			imu_sample_new.delta_ang_dt = _imu_dt_us * 1.e-6f;
			imu_sample_new.delta_ang = ang_vel * imu_sample_new.delta_ang_dt;
			imu_sample_new.delta_vel_dt = _imu_dt_us * 1.e-6f;
			imu_sample_new.delta_vel = accel * imu_sample_new.delta_vel_dt;

			_ekf->setIMUData(imu_sample_new);
			_counter_imu++;
		}
		if(_fuse_baro && !(_t_us % _baro_dt_us))
		{
			_ekf->setBaroData(_t_us,baro_data);
			_counter_baro++;
		}
		if(_fuse_mag && !(_t_us % _mag_dt_us))
		{
			float mag[3];
			mag_data.copyTo(mag);
			_ekf->setMagData(_t_us,mag);
			_counter_mag++;
		}
		if(_fuse_gps && !(_t_us % _gps_dt_us))
		{
			_gps_message.time_usec = _t_us;
			_ekf->setGpsData(_t_us,_gps_message);
			_counter_mag++;
		}

		_ekf->update();
	}
}

} // end of namespace simulator
