#include "ekf_wrapper.h"

EkfWrapper::EkfWrapper(std::shared_ptr<Ekf> ekf):
_ekf{ekf}
{
	_ekf_params = _ekf->getParamHandle();
}

EkfWrapper::~EkfWrapper()
{
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
