#include "sensor_simulator.h"


SensorSimulator::SensorSimulator(std::shared_ptr<Ekf> ekf):
	_airspeed(ekf),
	_baro(ekf),
	_flow(ekf),
	_gps(ekf),
	_imu(ekf),
	_mag(ekf),
	_rng(ekf),
	_vio(ekf),
	_ekf{ekf}
{
	setSensorRateToDefault();
	setSensorDataToDefault();
	startBasicSensor();
}

SensorSimulator::~SensorSimulator()
{

}

void SensorSimulator::loadSensorDataFromFile(std::string file_name)
{
	std::ifstream file(file_name);
	std::string line;

	while (!file.eof()) {
		std::string timestamp;
		std::string sensor_type;
		std::string sensor_data;
		sensor_info sensor_sample;

		getline(file, timestamp, ',');

		if (!timestamp.compare("")){ // empty line at end of file
			break;
		}
		sensor_sample.timestamp = std::stoul(timestamp);

		if(_replay_data.size() > 0) {
			sensor_info last_sample = _replay_data.back();
			if (sensor_sample.timestamp < last_sample.timestamp)
			{
				std::cout << "Timestamps not sorted ascendingly" << std::endl;
				exit(-1);
			}
		}

		getline(file, sensor_type, ',');
		if (!sensor_type.compare("imu")) {
			sensor_sample.sensor_type = sensor_info::IMU;
		} else if (!sensor_type.compare("mag")) {
			sensor_sample.sensor_type = sensor_info::MAG;

		} else if (!sensor_type.compare("baro")) {
			sensor_sample.sensor_type = sensor_info::BARO;

		} else if (!sensor_type.compare("gps")) {
			sensor_sample.sensor_type = sensor_info::GPS;

		} else if (!sensor_type.compare("airspeed")) {
			sensor_sample.sensor_type = sensor_info::AIRSPEED;

		} else if (!sensor_type.compare("range")) {
			sensor_sample.sensor_type = sensor_info::RANGE;

		} else if (!sensor_type.compare("flow")) {
			sensor_sample.sensor_type = sensor_info::FLOW;

		} else if (!sensor_type.compare("vio")) {
			sensor_sample.sensor_type = sensor_info::VISION;

		} else if (!sensor_type.compare("landed")) {
			sensor_sample.sensor_type = sensor_info::LANDING_STATUS;

		} else {
			std::cout << "Sensor type in file unknown" << std::endl;
			exit(-1);
		}

		getline(file, sensor_data);
		std::stringstream ss(sensor_data);
		int8_t i = 0;
		while( ss.good() )
		{
			if(i>=10){
				std::cout << "sensor data bigger than expected" << std::endl;
				exit(-1);
			}
			std::string value_string;
			getline( ss, value_string, ',' );
			if(!value_string.compare("")){
				continue;
			}
			sensor_sample.sensor_data[i] = std::stod(value_string);
			i++;
		}
		_replay_data.emplace_back(sensor_sample);
	}
	file.close();
	_has_replay_data = true;
}

void SensorSimulator::setSensorRateToDefault()
{
	_imu.setRateHz(200);
	_mag.setRateHz(80);
	_baro.setRateHz(80);
	_gps.setRateHz(5);
	_flow.setRateHz(50);
	_rng.setRateHz(30);
	_vio.setRateHz(30);
	_airspeed.setRateHz(100);
}

void SensorSimulator::setSensorDataToDefault()
{
	_airspeed.setData(0.0f, 0.0f);
	_baro.setData(122.2f);
	_flow.setData(_flow.dataAtRest());
	_gps.setData(_gps.getDefaultGpsData());
	_imu.setData(Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G}, Vector3f{0.0f,0.0f,0.0f});
	_mag.setData(Vector3f{0.2f, 0.0f, 0.4f});
	_rng.setData(0.2f, 100);
	_vio.setData(_vio.dataAtRest());
}

void SensorSimulator::startBasicSensor()
{
	_baro.start();
	_imu.start();
	_mag.start();
}

void SensorSimulator::runSeconds(float duration_seconds)
{
	runMicroseconds( uint32_t(duration_seconds * 1e6f) );
}

void SensorSimulator::runMicroseconds(uint32_t duration)
{
	// simulate in 1000us steps
	const uint64_t start_time = _time;

	for(;_time < start_time + (uint64_t)duration; _time+=1000)
	{
		bool update_imu = _imu.should_send(_time);
		updateSensors();

		if (update_imu) {
			// Update at IMU rate
			_ekf->update();
		}
	}
}

void SensorSimulator::updateSensors()
{
	_imu.update(_time);
	_mag.update(_time);
	_baro.update(_time);
	_gps.update(_time);
	_flow.update(_time);
	_rng.update(_time);
	_vio.update(_time);
	_airspeed.update(_time);
}

void SensorSimulator::runReplaySeconds(float duration_seconds)
{
	runReplayMicroseconds( uint32_t(duration_seconds * 1e6f) );
}

void SensorSimulator::runReplayMicroseconds(uint32_t duration)
{
	if(!_has_replay_data) {
		std::cout << "Can not run replay without replay data" << std::endl;
		exit(-1);
	}
	// simulate in 1000us steps
	const uint64_t start_time = _time;

	for(;_time < start_time + duration; _time+=1000)
	{
		setSensorDataFromReplayData();

		bool update_imu = _imu.should_send(_time);
		updateSensors();

		if(update_imu)
		{
			_ekf->update();
		}
	}
}

void SensorSimulator::setSensorDataFromReplayData()
{
	if(_replay_data.size() > 0) {
		sensor_info sample = _replay_data[_current_replay_data_index];
		while(sample.timestamp < _time)
		{
			setSingleReplaySample(sample);
			if(_current_replay_data_index < _replay_data.size())
			{
				_current_replay_data_index ++;
			} else {
				break;
			}
			sample = _replay_data[_current_replay_data_index];
		}
	} else {
		std::cerr << "Loaded replay data empty. Likely could not load replay data" << std::endl;
		exit(-1);
	}
}

void SensorSimulator::setSingleReplaySample(const sensor_info& sample)
{
	if (sample.sensor_type == sensor_info::IMU) {
		Vector3f accel{(float) sample.sensor_data[0],
				(float) sample.sensor_data[1],
				(float) sample.sensor_data[2]};
		Vector3f gyro{(float) sample.sensor_data[3],
				(float) sample.sensor_data[4],
				(float) sample.sensor_data[5]};
		_imu.setData(accel, gyro);

	} else if (sample.sensor_type == sensor_info::MAG) {
		Vector3f mag{(float) sample.sensor_data[0],
				(float) sample.sensor_data[1],
				(float) sample.sensor_data[2]};
		_mag.setData(mag);

	} else if (sample.sensor_type == sensor_info::BARO) {
		_baro.setData((float) sample.sensor_data[0]);

	} else if (sample.sensor_type == sensor_info::GPS) {
		_gps.setAltitude((int32_t) sample.sensor_data[0]);
		_gps.setLatitude((int32_t) sample.sensor_data[1]);
		_gps.setLongitude((int32_t) sample.sensor_data[2]);
		_gps.setVelocity(Vector3f((float) sample.sensor_data[3],
					  (float) sample.sensor_data[4],
					  (float) sample.sensor_data[5]));

	} else if (sample.sensor_type == sensor_info::AIRSPEED) {
			_airspeed.setData((float) sample.sensor_data[0], (float) sample.sensor_data[1]);

	} else if (sample.sensor_type == sensor_info::RANGE) {
			_rng.setData((float) sample.sensor_data[0], (float) sample.sensor_data[1]);

	} else if (sample.sensor_type == sensor_info::FLOW) {
		flowSample flow_sample;
		flow_sample.flow_xy_rad = Vector2f(sample.sensor_data[0],
						 sample.sensor_data[1]);
		flow_sample.gyro_xyz = Vector3f(sample.sensor_data[2],
					       sample.sensor_data[3],
					       sample.sensor_data[4]);
		flow_sample.quality = sample.sensor_data[5];
		_flow.setData(flow_sample);

	} else if (sample.sensor_type == sensor_info::VISION) {
		// sensor not yet implemented

		// extVisionSample vision_sample;
		// vision_sample.pos;
		// vision_sample.quat;
		// vision_sample.vel;
		// _vio.setData((float) sample.sensor_data[0], (float) sample.sensor_data[1]);

	} else if (sample.sensor_type == sensor_info::LANDING_STATUS) {
		bool landed = sample.sensor_data[0];
		_ekf->set_in_air_status(!landed);

	} else {
		printf("Unknown sensor type, can not set replay sample");
		exit(-1);
	}
}

void SensorSimulator::setGpsLatitude(const double latitude)
{
	int32_t lat = static_cast<int32_t>(latitude * 1e7);
	_gps.setLatitude(lat);
}

void SensorSimulator::setGpsLongitude(const double longitude)
{
	int32_t lon = static_cast<int32_t>(longitude * 1e7);
	_gps.setLongitude(lon);
}

void SensorSimulator::setGpsAltitude(const float altitude)
{
	int32_t alt = static_cast<int32_t>(altitude * 1e3);
	_gps.setAltitude(alt);
}

void SensorSimulator::setImuBias(Vector3f accel_bias, Vector3f gyro_bias)
{
	_imu.setData(Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G} + accel_bias,
		     Vector3f{0.0f,0.0f,0.0f} + gyro_bias);
}

void SensorSimulator::simulateOrientation(Quatf orientation)
{
	const Vector3f world_sensed_gravity = {0.0f, 0.0f, -CONSTANTS_ONE_G};
	const Vector3f world_mag_field = Vector3f{0.2f, 0.0f, 0.4f};
	const Dcmf R_bodyToWorld(orientation);
	const Vector3f sensed_gravity_body = R_bodyToWorld.transpose() * world_sensed_gravity;
	const Vector3f body_mag_field = R_bodyToWorld.transpose() * world_mag_field;

	_imu.setData(sensed_gravity_body, Vector3f{0.0f,0.0f,0.0f});
	_mag.setData(body_mag_field);
}
