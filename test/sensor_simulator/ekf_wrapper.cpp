#include "ekf_wrapper.h"

EkfWrapper::EkfWrapper(std::shared_ptr<Ekf> ekf):
_ekf{ekf}
{
	_ekf_params = _ekf->getParamHandle();
}

EkfWrapper::~EkfWrapper()
{
}

void EkfWrapper::setBaroHeight()
{
	_ekf_params->vdist_sensor_type = VDIST_SENSOR_BARO;
}

bool EkfWrapper::isIntendingBaroHeightFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.baro_hgt;
}

void EkfWrapper::setGpsHeight()
{
	_ekf_params->vdist_sensor_type = VDIST_SENSOR_GPS;
}

bool EkfWrapper::isIntendingGpsHeightFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.gps_hgt;
}

void EkfWrapper::setRangeHeight()
{
	_ekf_params->vdist_sensor_type = VDIST_SENSOR_RANGE;
}

bool EkfWrapper::isIntendingRangeHeightFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.rng_hgt;
}

void EkfWrapper::setVisionHeight()
{
	_ekf_params->vdist_sensor_type = VDIST_SENSOR_EV;
}

bool EkfWrapper::isIntendingVisionHeightFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.ev_hgt;
}

void EkfWrapper::enableGpsFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_GPS;
}

void EkfWrapper::disableGpsFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_GPS;
}

bool EkfWrapper::isIntendingGpsFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.gps;
}

void EkfWrapper::enableFlowFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_OF;
}

void EkfWrapper::disableFlowFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_OF;
}

bool EkfWrapper::isIntendingFlowFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.opt_flow;
}

void EkfWrapper::enableExternalVisionPositionFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_EVPOS;
}

void EkfWrapper::disableExternalVisionPositionFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_EVPOS;
}

bool EkfWrapper::isIntendingExternalVisionPositionFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.ev_pos;
}

void EkfWrapper::enableExternalVisionVelocityFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_EVVEL;
}

void EkfWrapper::disableExternalVisionVelocityFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_EVVEL;
}

bool EkfWrapper::isIntendingExternalVisionVelocityFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.ev_vel;
}

void EkfWrapper::enableExternalVisionHeadingFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_EVYAW;
}

void EkfWrapper::disableExternalVisionHeadingFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_EVYAW;
}

bool EkfWrapper::isIntendingExternalVisionHeadingFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.ev_yaw;
}

void EkfWrapper::enableExternalVisionAlignment()
{
	_ekf_params->fusion_mode |= MASK_ROTATE_EV;
}

void EkfWrapper::disableExternalVisionAlignment()
{
	_ekf_params->fusion_mode &= ~MASK_ROTATE_EV;
}

bool EkfWrapper::isWindVelocityEstimated() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.wind;
}

Vector3f EkfWrapper::getPosition() const
{
	float temp[3];
	_ekf->get_position(temp);
	return Vector3f(temp);
}
Vector3f EkfWrapper::getVelocity() const
{
	float temp[3];
	_ekf->get_velocity(temp);
	return Vector3f(temp);
}
Vector3f EkfWrapper::getAccelBias() const
{
	float temp[3];
	_ekf->get_accel_bias(temp);
	return Vector3f(temp);
}

Vector3f EkfWrapper::getGyroBias() const
{
	float temp[3];
	_ekf->get_gyro_bias(temp);
	return Vector3f(temp);
}

Quatf EkfWrapper::getQuaternion() const
{
	return _ekf->get_quaternion();
}

Eulerf EkfWrapper::getEulerAngles() const
{
	return Eulerf(getQuaternion());
}

matrix::Vector<float, 24> EkfWrapper::getState() const
{
	float state[24];
	_ekf->get_state_delayed(state);
	return matrix::Vector<float, 24>{state};
}

matrix::Vector<float, 4> EkfWrapper::getQuaternionVariance() const
{
	return matrix::Vector<float, 4>(_ekf->orientation_covariances().diag());
}

Vector3f EkfWrapper::getPositionVariance() const
{
	return Vector3f(_ekf->position_covariances().diag());
}

Vector3f EkfWrapper::getVelocityVariance() const
{
	return Vector3f(_ekf->velocity_covariances().diag());
}

Vector2f EkfWrapper::getWindVelocity() const
{
	float temp[2];
	_ekf->get_wind_velocity(temp);
	return Vector2f(temp);
}

Quatf EkfWrapper::getVisionAlignmentQuaternion() const
{
	float temp[4];
	_ekf->get_ev2ekf_quaternion(temp);
	return Quatf(temp);
}
