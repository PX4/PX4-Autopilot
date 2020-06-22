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

void EkfWrapper::enableGpsHeadingFusion()
{
	_ekf_params->fusion_mode |= MASK_USE_GPSYAW;
}

void EkfWrapper::disableGpsHeadingFusion()
{
	_ekf_params->fusion_mode &= ~MASK_USE_GPSYAW;
}

bool EkfWrapper::isIntendingGpsHeadingFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.gps_yaw;
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

bool EkfWrapper::isIntendingMagHeadingFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.mag_hdg;
}

bool EkfWrapper::isIntendingMag3DFusion() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.mag_3D;
}

bool EkfWrapper::isWindVelocityEstimated() const
{
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	return control_status.flags.wind;
}

void EkfWrapper::enableTerrainRngFusion()
{
	_ekf_params->terrain_fusion_mode |= TerrainFusionMask::TerrainFuseRangeFinder;
}

void EkfWrapper::disableTerrainRngFusion()
{
	_ekf_params->terrain_fusion_mode &= ~TerrainFusionMask::TerrainFuseRangeFinder;
}

bool EkfWrapper::isIntendingTerrainRngFusion() const
{
	terrain_fusion_status_u terrain_status;
	terrain_status.value = _ekf->getTerrainEstimateSensorBitfield();
	return terrain_status.flags.range_finder;
}

void EkfWrapper::enableTerrainFlowFusion()
{
	_ekf_params->terrain_fusion_mode |= TerrainFusionMask::TerrainFuseOpticalFlow;
}

void EkfWrapper::disableTerrainFlowFusion()
{
	_ekf_params->terrain_fusion_mode &= ~TerrainFusionMask::TerrainFuseOpticalFlow;
}

bool EkfWrapper::isIntendingTerrainFlowFusion() const
{
	terrain_fusion_status_u terrain_status;
	terrain_status.value = _ekf->getTerrainEstimateSensorBitfield();
	return terrain_status.flags.flow;
}

Eulerf EkfWrapper::getEulerAngles() const
{
	return Eulerf(_ekf->getQuaternion());
}

float EkfWrapper::getYawAngle() const
{
	const Eulerf euler(_ekf->getQuaternion());
	return euler(2);
}

matrix::Vector<float, 4> EkfWrapper::getQuaternionVariance() const
{
	return matrix::Vector<float, 4>(_ekf->orientation_covariances().diag());
}

int EkfWrapper::getQuaternionResetCounter() const
{
	float tmp[4];
	uint8_t counter;
	_ekf->get_quat_reset(tmp, &counter);
	return static_cast<int>(counter);
}

matrix::Vector3f EkfWrapper::getDeltaVelBiasVariance() const
{
	return _ekf->covariances_diagonal().slice<3, 1>(13,0);
}
