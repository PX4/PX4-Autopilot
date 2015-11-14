#pragma once

#include <controllib/estimators/BlockAttEst.hpp>
#include <controllib/blocks.hpp>

using namespace control;

class BlockAttEstQ : public BlockAttEst {
public:
	BlockAttEstQ();
	void correctMagAccel(const Vec6 & y, const SMat6 & R);
	void predict(const Vec6 & y, const SMat6 & R);
	static Vec4 f_dynamics(float t,
		const Matrix<float, 4, 1> &q_nb,
		const Matrix<float, 3, 1> &omega_nb);
private:
	// params
	BlockParamFloat _w_acc;
	BlockParamFloat _w_mag;
	BlockParamFloat _w_hdg;
	BlockParamFloat _w_ext_hdg;
	BlockParamFloat _w_gyro_bias;
	BlockParamFloat _mag_decl;
	BlockParamFloat _mag_decl_a;
	BlockParamInt _ext_hdg_m;
	BlockParamInt _acc_comp;
	BlockParamFloat _bias_max;
	BlockParamFloat _vibe_thresh;
	BlockLowPass2 _lp_roll_rate;
	BlockLowPass2 _lp_pitch_rate;
	BlockLowPass2 _lp_yaw_rate;
	// data
	Quatf _q_nb;
	Vec3 _b;
};
