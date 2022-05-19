#include "ekf_wrapper.h"

EkfWrapper::EkfWrapper(std::shared_ptr<Ekf> ekf):
	_ekf{ekf}
{
	_ekf_params = _ekf->getParamHandle();
}

EkfWrapper::~EkfWrapper()
{
}

void EkfWrapper::setBaroHeightRef()
{
	_ekf_params->height_sensor_ref |= HeightSensorRef::BARO;
}

void EkfWrapper::enableBaroHeightFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_BARO_HGT;
}

void EkfWrapper::disableBaroHeightFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_BARO_HGT;
}

bool EkfWrapper::isIntendingBaroHeightFusion() const
{
	return _ekf->control_status_flags().baro_hgt;
}

void EkfWrapper::setGpsHeightRef()
{
	_ekf_params->height_sensor_ref |= HeightSensorRef::GPS;
}

void EkfWrapper::enableGpsHeightFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_GPS_HGT;
}

void EkfWrapper::disableGpsHeightFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_GPS_HGT;
}

bool EkfWrapper::isIntendingGpsHeightFusion() const
{
	return _ekf->control_status_flags().gps_hgt;
}

void EkfWrapper::setRangeHeightRef()
{
	_ekf_params->height_sensor_ref |= HeightSensorRef::RANGE;
}

void EkfWrapper::enableRangeHeightFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_RNG_HGT;
}

void EkfWrapper::disableRangeHeightFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_RNG_HGT;
}

bool EkfWrapper::isIntendingRangeHeightFusion() const
{
	return _ekf->control_status_flags().rng_hgt;
}

void EkfWrapper::setVisionHeightRef()
{
	_ekf_params->height_sensor_ref |= HeightSensorRef::EV;
}

void EkfWrapper::enableVisionHeightFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_EXT_VIS_HGT;
}

void EkfWrapper::disableVisionHeightFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_EXT_VIS_HGT;
}

bool EkfWrapper::isIntendingVisionHeightFusion() const
{
	return _ekf->control_status_flags().ev_hgt;
}

void EkfWrapper::enableGpsFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_GPS;
}

void EkfWrapper::disableGpsFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_GPS;
}

bool EkfWrapper::isIntendingGpsFusion() const
{
	return _ekf->control_status_flags().gps;
}

void EkfWrapper::enableGpsHeadingFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_GPS_YAW;
}

void EkfWrapper::disableGpsHeadingFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_GPS_YAW;
}

bool EkfWrapper::isIntendingGpsHeadingFusion() const
{
	return _ekf->control_status_flags().gps_yaw;
}

void EkfWrapper::enableFlowFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_OPT_FLOW;
}

void EkfWrapper::disableFlowFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_OPT_FLOW;
}

bool EkfWrapper::isIntendingFlowFusion() const
{
	return _ekf->control_status_flags().opt_flow;
}

void EkfWrapper::setFlowOffset(const Vector3f &offset)
{
	_ekf_params->flow_pos_body = offset;
}

void EkfWrapper::enableExternalVisionPositionFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_EXT_VIS_POS;
}

void EkfWrapper::disableExternalVisionPositionFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_EXT_VIS_POS;
}

bool EkfWrapper::isIntendingExternalVisionPositionFusion() const
{
	return _ekf->control_status_flags().ev_pos;
}

void EkfWrapper::enableExternalVisionVelocityFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_EXT_VIS_VEL;
}

void EkfWrapper::disableExternalVisionVelocityFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_EXT_VIS_VEL;
}

bool EkfWrapper::isIntendingExternalVisionVelocityFusion() const
{
	return _ekf->control_status_flags().ev_vel;
}

void EkfWrapper::enableExternalVisionHeadingFusion()
{
	_ekf_params->fusion_mode |= SensorFusionMask::USE_EXT_VIS_YAW;
}

void EkfWrapper::disableExternalVisionHeadingFusion()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::USE_EXT_VIS_YAW;
}

bool EkfWrapper::isIntendingExternalVisionHeadingFusion() const
{
	return _ekf->control_status_flags().ev_yaw;
}

void EkfWrapper::enableExternalVisionAlignment()
{
	_ekf_params->fusion_mode |= SensorFusionMask::ROTATE_EXT_VIS;
}

void EkfWrapper::disableExternalVisionAlignment()
{
	_ekf_params->fusion_mode &= ~SensorFusionMask::ROTATE_EXT_VIS;
}

bool EkfWrapper::isIntendingMagHeadingFusion() const
{
	return _ekf->control_status_flags().mag_hdg;
}

bool EkfWrapper::isIntendingMag3DFusion() const
{
	return _ekf->control_status_flags().mag_3D;
}

void EkfWrapper::setMagFuseTypeNone()
{
	_ekf_params->mag_fusion_type = MagFuseType::NONE;
}

bool EkfWrapper::isWindVelocityEstimated() const
{
	return _ekf->control_status_flags().wind;
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
	return _ekf->covariances_diagonal().slice<3, 1>(13, 0);
}
