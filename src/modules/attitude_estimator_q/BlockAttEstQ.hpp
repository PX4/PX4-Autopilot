#pragma once

#include <controllib/estimators/BlockAttEst.hpp>
#include <controllib/blocks.hpp>

using namespace control;

class BlockAttEstQ : public BlockAttEst {
public:
	BlockAttEstQ();
	void update();
	void correctMagAccel(const Vec6 & y, const SMat6 & R);
	void predict(const Vec6 & y, const SMat6 & R);
private:
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
};
