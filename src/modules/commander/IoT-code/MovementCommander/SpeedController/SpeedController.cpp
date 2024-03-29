#include "SpeedController.hpp"


SpeedController::SpeedController(){
	Sens_sub = orb_subscribe(ORB_ID(sensor_combined));
    	orb_set_interval(Sens_sub, 100); // callback interval

	comb_fds[0].fd = Sens_sub;
    	comb_fds[0].events = POLLIN;
}


SpeedController::~SpeedController(){
	orb_unsubscribe(Sens_sub);
}

float* SpeedController::getPostion(){
	SpeedController::update();
	return positions;
}

float* SpeedController::getVelocities(){
	SpeedController::update();
	return velocities;
}

bool SpeedController::update(){
	int poll_ret = px4_poll(comb_fds, 1, 200);

	if (poll_ret == 0) {
	PX4_WARN("Timeout: No data received");
	return false;

	} else if(poll_ret < 0) {
	PX4_ERR("Error: poll failed");
	return false;
	}

    	orb_copy(ORB_ID(sensor_combined), Sens_sub, &Sens_st);

	hrt_abstime timestamp = Sens_st.timestamp;
	float dt = (timestamp - prev_timestamp) / 1e6f; // Convert microseconds to seconds

	// Integrate accelerometer data to estimate velocity
	for(int i =0; i<3 ; i++){
		velocities[i] += Sens_st.accelerometer_m_s2[i] * dt;
		positions[i] += velocities[i] * dt;
	}

	prev_timestamp = timestamp;
	return true;
}







