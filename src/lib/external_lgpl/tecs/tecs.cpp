// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "tecs.h"
#include <ecl/ecl.h>
#include <systemlib/err.h>
#include <geo/geo.h>

using namespace math;

/**
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 *
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

void TECS::update_state(float baro_altitude, float airspeed, const math::Matrix<3,3> &rotMat,
	const math::Vector<3> &accel_body, const math::Vector<3> &accel_earth, bool altitude_lock, bool in_air)
{
	// Implement third order complementary filter for height and height rate
	// estimted height rate = _integ2_state
	// estimated height     = _integ3_state
	// Reference Paper :
	// Optimising the Gains of the Baro-Inertial Vertical Channel
	// Widnall W.S, Sinha P.K,
	// AIAA Journal of Guidance and Control, 78-1307R

	// Calculate time in seconds since last update
	uint64_t now = ecl_absolute_time();
	float DT = max((now - _update_50hz_last_usec), 0ULL) * 1.0e-6f;

	// printf("dt: %10.6f baro alt: %6.2f eas: %6.2f R(0,0): %6.2f, R(1,1): %6.2f\naccel body: %6.2f %6.2f %6.2f\naccel earth: %6.2f %6.2f %6.2f\n",
	// 	DT, baro_altitude, airspeed, rotMat(0, 0), rotMat(1, 1), accel_body(0), accel_body(1), accel_body(2),
	// 	accel_earth(0), accel_earth(1), accel_earth(2));

	bool reset_altitude = false;

	if (_update_50hz_last_usec == 0 || DT > DT_MAX) {
		DT = DT_DEFAULT; // when first starting TECS, use small time constant
		reset_altitude = true;
	}

	if (!altitude_lock || !in_air) {
		reset_altitude = true;
	}

	if (reset_altitude) {
		_integ3_state = baro_altitude;
		_integ2_state = 0.0f;
		_integ1_state = 0.0f;

		// Reset the filter state as we just switched from non-altitude control
		// to altitude control mode
		_states_initalized = false;
	}

	_update_50hz_last_usec = now;
	_EAS = airspeed;

	_in_air = in_air;

	// Get height acceleration
	float hgt_ddot_mea = -(accel_earth(2) + CONSTANTS_ONE_G);
	// Perform filter calculation using backwards Euler integration
	// Coefficients selected to place all three filter poles at omega
	float omega2 = _hgtCompFiltOmega * _hgtCompFiltOmega;
	float hgt_err = baro_altitude - _integ3_state;
	float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
	_integ1_state = _integ1_state + integ1_input * DT;
	float integ2_input = _integ1_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
	_integ2_state = _integ2_state + integ2_input * DT;
	float integ3_input = _integ2_state + hgt_err * _hgtCompFiltOmega * 3.0f;

	// If more than 1 second has elapsed since last update then reset the integrator state
	// to the measured height
	if (reset_altitude) {
		_integ3_state = baro_altitude;

	} else {
		_integ3_state = _integ3_state + integ3_input * DT;
	}

	// Update and average speed rate of change
	// Only required if airspeed is being measured and controlled
	float temp = 0;

	if (PX4_ISFINITE(airspeed) && airspeed_sensor_enabled()) {
		// Get DCM
		// Calculate speed rate of change
		// XXX check
		temp = rotMat(2, 0) * CONSTANTS_ONE_G + accel_body(0);
		// take 5 point moving average
		//_vel_dot = _vdot_filter.apply(temp);
		// XXX resolve this properly
		_vel_dot = 0.95f * _vel_dot + 0.05f * temp;

	} else {
		_vel_dot = 0.0f;
	}

	if (!_in_air) {
		_states_initalized = false;
	}

}

void TECS::_update_speed(float airspeed_demand, float indicated_airspeed,
			 float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS)
{
	// Calculate time in seconds since last update
	uint64_t now = ecl_absolute_time();
	float DT = max((now - _update_speed_last_usec), 0ULL) * 1.0e-6f;

	// Convert equivalent airspeeds to true airspeeds

	_EAS_dem = airspeed_demand;
	_TAS_dem  = _EAS_dem * EAS2TAS;
	_TASmax   = indicated_airspeed_max * EAS2TAS;
	_TASmin   = indicated_airspeed_min * EAS2TAS;

	// Get airspeed or default to halfway between min and max if
	// airspeed is not being used and set speed rate to zero
	if (!PX4_ISFINITE(indicated_airspeed) || !airspeed_sensor_enabled()) {
		// If no airspeed available use average of min and max
		_EAS = 0.5f * (indicated_airspeed_min + indicated_airspeed_max);

	} else {
		_EAS = indicated_airspeed;
	}

	// Reset states on initial execution or if not active
	if (_update_speed_last_usec == 0 || !_in_air) {
		_integ4_state = 0.0f;
		_integ5_state = (_EAS * EAS2TAS);
	}

	if (DT < DT_MIN || DT > DT_MAX) {
		DT = DT_DEFAULT; // when first starting TECS, use small time constant
	}

	// Implement a second order complementary filter to obtain a
	// smoothed airspeed estimate
	// airspeed estimate is held in _integ5_state
	float aspdErr = (_EAS * EAS2TAS) - _integ5_state;
	float integ4_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;

	// Prevent state from winding up
	if (_integ5_state < 3.1f) {
		integ4_input = max(integ4_input , 0.0f);
	}

	_integ4_state = _integ4_state + integ4_input * DT;
	float integ5_input = _integ4_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
	_integ5_state = _integ5_state + integ5_input * DT;
	// limit the airspeed to a minimum of 3 m/s
	_integ5_state = max(_integ5_state, 3.0f);
	_update_speed_last_usec = now;
}

void TECS::_update_speed_demand(void)
{
	// Set the airspeed demand to the minimum value if an underspeed condition exists
	// or a bad descent condition exists
	// This will minimise the rate of descent resulting from an engine failure,
	// enable the maximum climb rate to be achieved and prevent continued full power descent
	// into the ground due to an unachievable airspeed value
	if ((_badDescent) || (_underspeed)) {
		_TAS_dem     = _TASmin;
	}

	// Constrain speed demand
	_TAS_dem = constrain(_TAS_dem, _TASmin, _TASmax);

	// calculate velocity rate limits based on physical performance limits
	// provision to use a different rate limit if bad descent or underspeed condition exists
	// Use 50% of maximum energy rate to allow margin for total energy controller
	float velRateMax;
	float velRateMin;

	if ((_badDescent) || (_underspeed)) {
		velRateMax = 0.5f * _STEdot_max / _integ5_state;
		velRateMin = 0.5f * _STEdot_min / _integ5_state;

	} else {
		velRateMax = 0.5f * _STEdot_max / _integ5_state;
		velRateMin = 0.5f * _STEdot_min / _integ5_state;
	}

//	// Apply rate limit
//	if ((_TAS_dem - _TAS_dem_adj) > (velRateMax * _DT)) {
//		_TAS_dem_adj = _TAS_dem_adj + velRateMax * _DT;
//		_TAS_rate_dem = velRateMax;
//
//	} else if ((_TAS_dem - _TAS_dem_adj) < (velRateMin * _DT)) {
//		_TAS_dem_adj = _TAS_dem_adj + velRateMin * _DT;
//		_TAS_rate_dem = velRateMin;
//
//	} else {
//		_TAS_dem_adj = _TAS_dem;
//
//
//	_TAS_rate_dem = (_TAS_dem - _TAS_dem_last) / _DT;
//	}

	_TAS_dem_adj = constrain(_TAS_dem, _TASmin, _TASmax);;
	_TAS_rate_dem = constrain((_TAS_dem_adj - _integ5_state) * _speedrate_p, velRateMin, velRateMax); //xxx: using a p loop for now

//	_TAS_dem_last = _TAS_dem;

//	warnx("_TAS_rate_dem: %.1f, _TAS_dem_adj %.1f, _integ5_state %.1f, _badDescent %u , _underspeed %u, velRateMin %.1f",
//			(double)_TAS_rate_dem, (double)_TAS_dem_adj, (double)_integ5_state, _badDescent, _underspeed, velRateMin);
//	warnx("_TAS_rate_dem: %.1f, _TAS_dem_adj %.1f, _integ5_state %.1f,  _badDescent %u , _underspeed %u",
//			(double)_TAS_rate_dem, (double)_TAS_dem_adj, (double)_integ5_state,  _badDescent , _underspeed);
}

void TECS::_update_height_demand(float demand, float state)
{
	// Handle initialization
	if (PX4_ISFINITE(demand) && fabsf(_hgt_dem_in_old) < 0.1f) {
		_hgt_dem_in_old = demand;
	}
	// Apply 2 point moving average to demanded height
	// This is required because height demand is updated in steps
	if (PX4_ISFINITE(demand)) {
		_hgt_dem = 0.5f * (demand + _hgt_dem_in_old);
	} else {
		_hgt_dem = _hgt_dem_in_old;
	}
	_hgt_dem_in_old = _hgt_dem;

	// Limit height demand
	// this is important to avoid a windup
	if ((_hgt_dem - _hgt_dem_prev) > (_maxClimbRate * _DT)) {
		_hgt_dem = _hgt_dem_prev + _maxClimbRate * _DT;

	} else if ((_hgt_dem - _hgt_dem_prev) < (-_maxSinkRate * _DT)) {
		_hgt_dem = _hgt_dem_prev - _maxSinkRate * _DT;
	}

	_hgt_dem_prev = _hgt_dem;
//
//	// Apply first order lag to height demand
//	_hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;
//	_hgt_rate_dem = (_hgt_dem_adj - _hgt_dem_adj_last) / 0.1f;
//	_hgt_dem_adj_last = _hgt_dem_adj;
//
//	// printf("hgt_dem: %6.2f hgt_dem_adj: %6.2f hgt_dem_last: %6.2f hgt_rate_dem: %6.2f\n", _hgt_dem, _hgt_dem_adj, _hgt_dem_adj_last,
//	// 	_hgt_rate_dem);

	_hgt_dem_adj = 0.1f * _hgt_dem + 0.9f * _hgt_dem_adj_last;
	_hgt_dem_adj_last = _hgt_dem_adj;
	_hgt_rate_dem = (_hgt_dem_adj - state) * _heightrate_p + _heightrate_ff * (_hgt_dem_adj - _hgt_dem_adj_last) / _DT;

	// Limit height rate of change
	if (_hgt_rate_dem > _maxClimbRate) {
		_hgt_rate_dem = _maxClimbRate;

	} else if (_hgt_rate_dem < -_maxSinkRate) {
		_hgt_rate_dem = -_maxSinkRate;
	}

	//warnx("_hgt_rate_dem: %.4f, _hgt_dem_adj %.4f", _hgt_rate_dem, _hgt_dem_adj);
}

void TECS::_detect_underspeed(void)
{
	if (!_detect_underspeed_enabled) {
		_underspeed = false;
		return;
	}

	if (((_integ5_state < _TASmin * 0.9f) && (_throttle_dem >= _THRmaxf * 0.95f)) || ((_integ3_state < _hgt_dem_adj) && _underspeed)) {
		_underspeed = true;

	} else {
		_underspeed = false;
	}
}

void TECS::_update_energies(void)
{
	// Calculate specific energy demands
	_SPE_dem = _hgt_dem_adj * CONSTANTS_ONE_G;
	_SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

	// Calculate specific energy rate demands
	_SPEdot_dem = _hgt_rate_dem * CONSTANTS_ONE_G;
	_SKEdot_dem = _integ5_state * _TAS_rate_dem;

	// Calculate specific energy
	_SPE_est = _integ3_state * CONSTANTS_ONE_G;
	_SKE_est = 0.5f * _integ5_state * _integ5_state;

	// Calculate specific energy rate
	_SPEdot = _integ2_state * CONSTANTS_ONE_G;
	_SKEdot = _integ5_state * _vel_dot;
}

void TECS::_update_throttle(float throttle_cruise, const math::Matrix<3,3> &rotMat)
{
	// Calculate total energy values
	_STE_error = _SPE_dem - _SPE_est + _SKE_dem - _SKE_est;
	float STEdot_dem = constrain((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
	_STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

	// Apply 0.5 second first order filter to STEdot_error
	// This is required to remove accelerometer noise from the  measurement
	_STEdot_error = 0.2f * _STEdot_error + 0.8f * _STEdotErrLast;
	_STEdotErrLast = _STEdot_error;

	// Calculate throttle demand
	// If underspeed condition is set, then demand full throttle
	if (_underspeed) {
		_throttle_dem = 1.0f;

	} else {
		// Calculate gain scaler from specific energy error to throttle
		float K_STE2Thr = 1 / (_timeConstThrot * (_STEdot_max - _STEdot_min));

		// Calculate feed-forward throttle
		float ff_throttle = 0;
		float nomThr = throttle_cruise;
		// Use the demanded rate of change of total energy as the feed-forward demand, but add
		// additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
		// drag increase during turns.
		float cosPhi = sqrtf((rotMat(0, 1) * rotMat(0, 1)) + (rotMat(1, 1) * rotMat(1, 1)));
		STEdot_dem = STEdot_dem + _rollComp * (1.0f / constrain(cosPhi , 0.1f, 1.0f) - 1.0f);

		if (STEdot_dem >= 0) {
			ff_throttle = nomThr + STEdot_dem / _STEdot_max * (_THRmaxf - nomThr);

		} else {
			ff_throttle = nomThr - STEdot_dem / _STEdot_min * nomThr;
		}

		// Calculate PD + FF throttle and constrain to avoid blow-up of the integrator later
		_throttle_dem = (_STE_error + _STEdot_error * _thrDamp) * K_STE2Thr + ff_throttle;
		_throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);

		// Rate limit PD + FF throttle
		// Calculate the throttle increment from the specified slew time
		if (fabsf(_throttle_slewrate) > 0.01f) {
			float thrRateIncr = _DT * (_THRmaxf - _THRminf) * _throttle_slewrate;
			_throttle_dem = constrain(_throttle_dem,
						_last_throttle_dem - thrRateIncr,
						_last_throttle_dem + thrRateIncr);
		}

		// Ensure _last_throttle_dem is always initialized properly
		_last_throttle_dem = _throttle_dem;

		// Calculate integrator state upper and lower limits
		// Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
		float integ_max = (_THRmaxf - _throttle_dem + 0.1f);
		float integ_min = (_THRminf - _throttle_dem - 0.1f);

		// Calculate integrator state, constraining state
		// Set integrator to a max throttle value during climbout
		_integ6_state = _integ6_state + (_STE_error * _integGain) * _DT * K_STE2Thr;

		if (_climbOutDem) {
			_integ6_state = integ_max;

		} else {
			_integ6_state = constrain(_integ6_state, integ_min, integ_max);
		}

		// Sum the components.
		// Only use feed-forward component if airspeed is not being used
		if (airspeed_sensor_enabled()) {
			_throttle_dem = _throttle_dem + _integ6_state;

		} else {
			_throttle_dem = ff_throttle;
		}

		// Constrain throttle demand
		_throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);
	}
}

void TECS::_detect_bad_descent(void)
{
	// Detect a demanded airspeed too high for the aircraft to achieve. This will be
	// evident by the the following conditions:
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 200 (greater than ~20m height error)
	// 3) Specific total energy reducing
	// 4) throttle demand > 90%
	// If these four conditions exist simultaneously, then the protection
	// mode will be activated.
	// Once active, the following condition are required to stay in the mode
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 0
	// This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
//	float STEdot = _SPEdot + _SKEdot;
//
//	if ((!_underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (_badDescent && !_underspeed && (_STE_error > 0.0f))) {
//
//		warnx("bad descent detected: _STE_error %.1f, STEdot %.1f, _throttle_dem %.1f", _STE_error, STEdot, _throttle_dem);
//		_badDescent = true;
//
//	} else {
//		_badDescent = false;
//	}

	_badDescent = false;
}

void TECS::_update_pitch(void)
{
	// Calculate Speed/Height Control Weighting
	// This is used to determine how the pitch control prioritises speed and height control
	// A weighting of 1 provides equal priority (this is the normal mode of operation)
	// A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
	// A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected
	// or during takeoff/climbout where a minimum pitch angle is set to ensure height is gained. In this instance, if airspeed
	// rises above the demanded value, the pitch angle will be increased by the TECS controller.
	float SKE_weighting = constrain(_spdWeight, 0.0f, 2.0f);

	if ((_underspeed || _climbOutDem) && airspeed_sensor_enabled()) {
		SKE_weighting = 2.0f;

	} else if (!airspeed_sensor_enabled()) {
		SKE_weighting = 0.0f;
	}

	float SPE_weighting = 2.0f - SKE_weighting;

	// Calculate Specific Energy Balance demand, and error
	float SEB_dem = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
	float SEBdot_dem = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
	_SEB_error = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
	_SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);

	// Calculate integrator state, constraining input if pitch limits are exceeded
	float integ7_input = _SEB_error * _integGain;

	if (_pitch_dem_unc > _PITCHmaxf) {
		integ7_input = min(integ7_input, _PITCHmaxf - _pitch_dem_unc);

	} else if (_pitch_dem_unc < _PITCHminf) {
		integ7_input = max(integ7_input, _PITCHminf - _pitch_dem_unc);
	}

	_integ7_state = _integ7_state + integ7_input * _DT;

	// Apply max and min values for integrator state that will allow for no more than
	// 5deg of saturation. This allows for some pitch variation due to gusts before the
	// integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
	// During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
	// demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
	// integrator has to catch up before the nose can be raised to reduce speed during climbout.
	float gainInv = (_integ5_state * _timeConst * CONSTANTS_ONE_G);
	float temp = _SEB_error + _SEBdot_error * _ptchDamp + SEBdot_dem * _timeConst;
	if (_climbOutDem)
	{
		temp += _PITCHminf * gainInv;
	}
	_integ7_state = constrain(_integ7_state, (gainInv * (_PITCHminf - 0.0783f)) - temp, (gainInv * (_PITCHmaxf + 0.0783f)) - temp);


	// Calculate pitch demand from specific energy balance signals
	_pitch_dem_unc = (temp + _integ7_state) / gainInv;

	// Constrain pitch demand
	_pitch_dem = constrain(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

	// Rate limit the pitch demand to comply with specified vertical
	// acceleration limit
	float ptchRateIncr = _DT * _vertAccLim / _integ5_state;

	if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
		_pitch_dem = _last_pitch_dem + ptchRateIncr;

	} else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
		_pitch_dem = _last_pitch_dem - ptchRateIncr;
	}

	_last_pitch_dem = _pitch_dem;
}

void TECS::_initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad)
{
	// Initialise states and variables if DT > 1 second or in climbout
	if (_update_pitch_throttle_last_usec == 0 || _DT > DT_MAX || !_in_air || !_states_initalized) {
		_integ1_state = 0.0f;
		_integ2_state = 0.0f;
		_integ3_state = baro_altitude;
		_integ4_state = 0.0f;
		_integ5_state = _EAS;
		_integ6_state = 0.0f;
		_integ7_state = 0.0f;
		_last_throttle_dem = throttle_cruise;
		_last_pitch_dem = pitch;
		_hgt_dem_adj_last = baro_altitude;
		_hgt_dem_adj = _hgt_dem_adj_last;
		_hgt_dem_prev = _hgt_dem_adj_last;
		_hgt_dem_in_old = _hgt_dem_adj_last;
		_TAS_dem_last = _TAS_dem;
		_TAS_dem_adj = _TAS_dem;
		_pitch_dem_unc = pitch;
		_underspeed = false;
		_badDescent = false;

		if (_DT > DT_MAX || _DT < DT_MIN) {
			_DT = DT_DEFAULT;
		}

	} else if (_climbOutDem) {
		_PITCHminf          = ptchMinCO_rad;
		_THRminf            = _THRmaxf - 0.01f;
		_hgt_dem_adj_last  = baro_altitude;
		_hgt_dem_adj       = _hgt_dem_adj_last;
		_hgt_dem_prev      = _hgt_dem_adj_last;
		_TAS_dem_last      = _TAS_dem;
		_TAS_dem_adj       = _TAS_dem;
		_underspeed        = false;
		_badDescent 	   = false;
	}

	_states_initalized = true;
}

void TECS::_update_STE_rate_lim(void)
{
	// Calculate Specific Total Energy Rate Limits
	// This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
	_STEdot_max = _maxClimbRate * CONSTANTS_ONE_G;
	_STEdot_min = - _minSinkRate * CONSTANTS_ONE_G;
}

void TECS::update_pitch_throttle(const math::Matrix<3,3> &rotMat, float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem, float ptchMinCO,
				 float throttle_min, float throttle_max, float throttle_cruise,
				 float pitch_limit_min, float pitch_limit_max)
{

	// Calculate time in seconds since last update
	uint64_t now = ecl_absolute_time();
	_DT = max((now - _update_pitch_throttle_last_usec), 0ULL) * 1.0e-6f;

	// printf("tecs in: dt:%10.6f pitch: %6.2f baro_alt: %6.2f alt sp: %6.2f\neas sp: %6.2f eas: %6.2f, eas2tas: %6.2f\n %s pitch min C0: %6.2f thr min: %6.2f, thr max: %6.2f thr cruis: %6.2f pt min: %6.2f, pt max: %6.2f\n",
	// 	_DT, pitch, baro_altitude, hgt_dem, EAS_dem, indicated_airspeed, EAS2TAS, (climbOutDem) ? "climb" : "level", ptchMinCO, throttle_min, throttle_max, throttle_cruise, pitch_limit_min, pitch_limit_max);

	// Convert inputs
	_THRmaxf = throttle_max;
	_THRminf = throttle_min;
	_PITCHmaxf = pitch_limit_max;
	_PITCHminf = pitch_limit_min;
	_climbOutDem = climbOutDem;

	// initialise selected states and variables if DT > 1 second or in climbout
	_initialise_states(pitch, throttle_cruise, baro_altitude, ptchMinCO);

	if (!_in_air) {
		return;
	}

	// Update the speed estimate using a 2nd order complementary filter
	_update_speed(EAS_dem, indicated_airspeed, _indicated_airspeed_min, _indicated_airspeed_max, EAS2TAS);

	// Calculate Specific Total Energy Rate Limits
	_update_STE_rate_lim();

	// Calculate the speed demand
	_update_speed_demand();

	// Calculate the height demand
	_update_height_demand(hgt_dem, baro_altitude);

	// Detect underspeed condition
	_detect_underspeed();

	// Calculate specific energy quantitiues
	_update_energies();

	// Calculate throttle demand
	_update_throttle(throttle_cruise, rotMat);

	// Detect bad descent due to demanded airspeed being too high
	_detect_bad_descent();

	// Calculate pitch demand
	_update_pitch();

	_tecs_state.timestamp = now;

	if (_underspeed) {
		_tecs_state.mode = ECL_TECS_MODE_UNDERSPEED;
	} else if (_badDescent) {
		_tecs_state.mode = ECL_TECS_MODE_BAD_DESCENT;
	} else if (_climbOutDem) {
		_tecs_state.mode = ECL_TECS_MODE_CLIMBOUT;
	} else {
		// If no error flag applies, conclude normal
		_tecs_state.mode = ECL_TECS_MODE_NORMAL;
	}

	_tecs_state.altitude_sp = _hgt_dem_adj;
	_tecs_state.altitude_filtered = _integ3_state;
	_tecs_state.altitude_rate_sp = _hgt_rate_dem;
	_tecs_state.altitude_rate = _integ2_state;

	_tecs_state.airspeed_sp = _TAS_dem_adj;
	_tecs_state.airspeed_rate_sp = _TAS_rate_dem;
	_tecs_state.airspeed_filtered = _integ5_state;
	_tecs_state.airspeed_rate = _vel_dot;

	_tecs_state.total_energy_error = _STE_error;
	_tecs_state.energy_distribution_error = _SEB_error;
	_tecs_state.total_energy_rate_error = _STEdot_error;
	_tecs_state.energy_distribution_rate_error = _SEBdot_error;

	_tecs_state.energy_error_integ = _integ6_state;
	_tecs_state.energy_distribution_error_integ = _integ7_state;

	_tecs_state.throttle_integ 	= _integ6_state;
	_tecs_state.pitch_integ 	= _integ7_state;

	_update_pitch_throttle_last_usec = now;

}
