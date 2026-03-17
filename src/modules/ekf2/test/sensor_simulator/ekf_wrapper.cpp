#include "ekf_wrapper.h"

EkfWrapper::EkfWrapper(std::shared_ptr<Ekf> ekf):
	_ekf{ekf}
{
	_ekf_params = _ekf->getParamHandle();
	_fc = _ekf->getFusionControlHandle();

	// Sync FusionControl from default param values so Ekf core sees them
	_fc->gps.enabled = true;
	_fc->gps.intended = static_cast<uint8_t>(_ekf_params->ekf2_gps_ctrl);
	_fc->of.enabled = true;
	_fc->of.intended = static_cast<uint8_t>(_ekf_params->ekf2_of_ctrl);
	_fc->ev.enabled = true;
	_fc->ev.intended = static_cast<uint8_t>(_ekf_params->ekf2_ev_ctrl);
	_fc->baro.enabled = true;
	_fc->baro.intended = static_cast<uint8_t>(_ekf_params->ekf2_baro_ctrl);
	_fc->rng.enabled = true;
	_fc->rng.intended = static_cast<uint8_t>(_ekf_params->ekf2_rng_ctrl);
	_fc->drag.enabled = true;
	_fc->drag.intended = static_cast<uint8_t>(_ekf_params->ekf2_drag_ctrl);
	_fc->mag.enabled = true;
	_fc->mag.intended = static_cast<uint8_t>(_ekf_params->ekf2_mag_type);
	_fc->aspd.enabled = true;
	_fc->aspd.intended = static_cast<uint8_t>(_ekf_params->ekf2_arsp_thr);
	_fc->rngbcn.enabled = true;
	_fc->rngbcn.intended = 0; // wait for RangeBeacon PR

	for (int i = 0; i < MAX_AGP_INSTANCES; i++) {
		_fc->agp[i].enabled = true;
		_fc->agp[i].intended = 0;
	}
}

EkfWrapper::~EkfWrapper()
{
}

void EkfWrapper::syncGpsFc()
{
	_fc->gps.enabled = true;
	_fc->gps.intended = static_cast<uint8_t>(_ekf_params->ekf2_gps_ctrl);
}

void EkfWrapper::syncEvFc()
{
	_fc->ev.enabled = true;
	_fc->ev.intended = static_cast<uint8_t>(_ekf_params->ekf2_ev_ctrl);
}

void EkfWrapper::setBaroHeightRef()
{
	_ekf_params->ekf2_hgt_ref = static_cast<int32_t>(HeightSensor::BARO);
}

void EkfWrapper::enableBaroHeightFusion()
{
	_ekf_params->ekf2_baro_ctrl = 1;
	_fc->baro.enabled = true;
	_fc->baro.intended = 1;
}

void EkfWrapper::disableBaroHeightFusion()
{
	_ekf_params->ekf2_baro_ctrl = 0;
	_fc->baro.intended = 0;
}

bool EkfWrapper::isIntendingBaroHeightFusion() const
{
	return _ekf->control_status_flags().baro_hgt;
}

void EkfWrapper::setGpsHeightRef()
{
	_ekf_params->ekf2_hgt_ref = static_cast<int32_t>(HeightSensor::GNSS);
}

void EkfWrapper::enableGpsHeightFusion()
{
	_ekf_params->ekf2_gps_ctrl |= static_cast<int32_t>(GnssCtrl::VPOS);
	syncGpsFc();
}

void EkfWrapper::disableGpsHeightFusion()
{
	_ekf_params->ekf2_gps_ctrl &= ~static_cast<int32_t>(GnssCtrl::VPOS);
	syncGpsFc();
}

bool EkfWrapper::isIntendingGpsHeightFusion() const
{
	return _ekf->control_status_flags().gps_hgt;
}

void EkfWrapper::setRangeHeightRef()
{
	_ekf_params->ekf2_hgt_ref = static_cast<int32_t>(HeightSensor::RANGE);
}

void EkfWrapper::enableRangeHeightFusion()
{
	_ekf_params->ekf2_rng_ctrl = static_cast<int32_t>(RngCtrl::ENABLED);
	_fc->rng.enabled = true;
	_fc->rng.intended = static_cast<uint8_t>(RngCtrl::ENABLED);
}

void EkfWrapper::disableRangeHeightFusion()
{
	_ekf_params->ekf2_rng_ctrl = static_cast<int32_t>(RngCtrl::DISABLED);
	_fc->rng.intended = static_cast<uint8_t>(RngCtrl::DISABLED);
}

bool EkfWrapper::isIntendingRangeHeightFusion() const
{
	return _ekf->control_status_flags().rng_hgt;
}

void EkfWrapper::setExternalVisionHeightRef()
{
	_ekf_params->ekf2_hgt_ref = static_cast<int32_t>(HeightSensor::EV);
}

void EkfWrapper::enableExternalVisionHeightFusion()
{
	_ekf_params->ekf2_ev_ctrl |= static_cast<int32_t>(EvCtrl::VPOS);
	syncEvFc();
}

bool EkfWrapper::isIntendingExternalVisionHeightFusion() const
{
	return _ekf->control_status_flags().ev_hgt;
}

void EkfWrapper::enableBetaFusion()
{
	_ekf_params->ekf2_fuse_beta = true;
}

void EkfWrapper::disableBetaFusion()
{
	_ekf_params->ekf2_fuse_beta = false;
}

bool EkfWrapper::isIntendingBetaFusion() const
{
	return _ekf->control_status_flags().fuse_beta;
}

bool EkfWrapper::isIntendingAirspeedFusion() const
{
	return _ekf->control_status_flags().fuse_aspd;
}

void EkfWrapper::enableGpsFusion()
{
	_ekf_params->ekf2_gps_ctrl |= static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL);
	syncGpsFc();
}

void EkfWrapper::disableGpsFusion()
{
	_ekf_params->ekf2_gps_ctrl &= ~(static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL));
	syncGpsFc();
}

bool EkfWrapper::isIntendingGpsFusion() const
{
	return _ekf->control_status_flags().gnss_vel || _ekf->control_status_flags().gnss_pos;
}

bool EkfWrapper::isGnssFaultDetected() const
{
	return _ekf->control_status_flags().gnss_fault;
}

void EkfWrapper::setGnssDeadReckonMode()
{
	_ekf_params->ekf2_gps_mode = static_cast<int32_t>(GnssMode::kDeadReckoning);
}

void EkfWrapper::enableGpsHeadingFusion()
{
	_ekf_params->ekf2_gps_ctrl |= static_cast<int32_t>(GnssCtrl::YAW);
	syncGpsFc();
}

void EkfWrapper::disableGpsHeadingFusion()
{
	_ekf_params->ekf2_gps_ctrl &= ~static_cast<int32_t>(GnssCtrl::YAW);
	syncGpsFc();
}

bool EkfWrapper::isIntendingGpsHeadingFusion() const
{
	return _ekf->control_status_flags().gnss_yaw;
}

void EkfWrapper::enableFlowFusion()
{
	_ekf_params->ekf2_of_ctrl = 1;
	_fc->of.enabled = true;
	_fc->of.intended = 1;
}

void EkfWrapper::disableFlowFusion()
{
	_ekf_params->ekf2_of_ctrl = 0;
	_fc->of.intended = 0;
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
	_ekf_params->ekf2_ev_ctrl |= static_cast<int32_t>(EvCtrl::HPOS);
	syncEvFc();
}

void EkfWrapper::disableExternalVisionPositionFusion()
{
	_ekf_params->ekf2_ev_ctrl &= ~static_cast<int32_t>(EvCtrl::HPOS);
	syncEvFc();
}

bool EkfWrapper::isIntendingExternalVisionPositionFusion() const
{
	return _ekf->control_status_flags().ev_pos;
}

void EkfWrapper::enableExternalVisionVelocityFusion()
{
	_ekf_params->ekf2_ev_ctrl |= static_cast<int32_t>(EvCtrl::VEL);
	syncEvFc();
}

void EkfWrapper::disableExternalVisionVelocityFusion()
{
	_ekf_params->ekf2_ev_ctrl &= ~static_cast<int32_t>(EvCtrl::VEL);
	syncEvFc();
}

bool EkfWrapper::isIntendingExternalVisionVelocityFusion() const
{
	return _ekf->control_status_flags().ev_vel;
}

void EkfWrapper::enableExternalVisionHeadingFusion()
{
	_ekf_params->ekf2_ev_ctrl |= static_cast<int32_t>(EvCtrl::YAW);
	syncEvFc();
}

void EkfWrapper::disableExternalVisionHeadingFusion()
{
	_ekf_params->ekf2_ev_ctrl &= ~static_cast<int32_t>(EvCtrl::YAW);
	syncEvFc();
}

bool EkfWrapper::isIntendingExternalVisionHeadingFusion() const
{
	return _ekf->control_status_flags().ev_yaw;
}

bool EkfWrapper::isIntendingMagFusion() const
{
	return _ekf->control_status_flags().mag;
}

bool EkfWrapper::isIntendingMagHeadingFusion() const
{
	return _ekf->control_status_flags().mag_hdg;
}

bool EkfWrapper::isIntendingMag3DFusion() const
{
	return _ekf->control_status_flags().mag_3D;
}

bool EkfWrapper::isMagHeadingConsistent() const
{
	return _ekf->control_status_flags().mag_heading_consistent;
}

bool EkfWrapper::isMagFaultDetected() const
{
	return _ekf->control_status_flags().mag_fault;
}

void EkfWrapper::setMagFuseTypeNone()
{
	_ekf_params->ekf2_mag_type = MagFuseType::NONE;
	_fc->mag.intended = static_cast<uint8_t>(MagFuseType::NONE);
}

void EkfWrapper::enableMagStrengthCheck()
{
	_ekf_params->ekf2_mag_check |= static_cast<int32_t>(MagCheckMask::STRENGTH);
}

void EkfWrapper::enableMagInclinationCheck()
{
	_ekf_params->ekf2_mag_check |= static_cast<int32_t>(MagCheckMask::INCLINATION);
}

void EkfWrapper::enableMagCheckForceWMM()
{
	_ekf_params->ekf2_mag_check |= static_cast<int32_t>(MagCheckMask::FORCE_WMM);
}

bool EkfWrapper::isWindVelocityEstimated() const
{
	return _ekf->control_status_flags().wind;
}

bool EkfWrapper::isIntendingTerrainRngFusion() const
{
	return _ekf->control_status_flags().rng_terrain;
}

bool EkfWrapper::isIntendingTerrainFlowFusion() const
{
	return _ekf->control_status_flags().opt_flow_terrain;
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

int EkfWrapper::getQuaternionResetCounter() const
{
	float tmp[4];
	uint8_t counter;
	_ekf->get_quat_reset(tmp, &counter);
	return static_cast<int>(counter);
}

void EkfWrapper::enableDragFusion()
{
	_ekf_params->ekf2_drag_ctrl = 1;
	_fc->drag.enabled = true;
	_fc->drag.intended = 1;
}

void EkfWrapper::disableDragFusion()
{
	_ekf_params->ekf2_drag_ctrl = 0;
	_fc->drag.intended = 0;
}

void EkfWrapper::setDragFusionParameters(const float &bcoef_x, const float &bcoef_y, const float &mcoef)
{
	_ekf_params->ekf2_bcoef_x = bcoef_x;
	_ekf_params->ekf2_bcoef_y = bcoef_y;
	_ekf_params->ekf2_mcoef = mcoef;
}

float EkfWrapper::getMagHeadingNoise() const
{
	return _ekf_params->ekf2_head_noise;
}

void EkfWrapper::enableGyroBiasEstimation()
{
	_ekf_params->ekf2_imu_ctrl |= static_cast<int32_t>(ImuCtrl::GyroBias);
}

void EkfWrapper::disableGyroBiasEstimation()
{
	_ekf_params->ekf2_imu_ctrl &= ~static_cast<int32_t>(ImuCtrl::GyroBias);
}
