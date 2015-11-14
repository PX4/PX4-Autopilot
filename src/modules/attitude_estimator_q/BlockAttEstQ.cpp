#include "BlockAttEstQ.hpp"

BlockAttEstQ::BlockAttEstQ() :
	BlockAttEst(nullptr, "ATT"),
	_w_acc(this, "W_ACC"),
	_w_mag(this, "W_ACC"),
	_w_hdg(this, "W_HDG"),
	_w_ext_hdg(this, "W_EXT_HDG"),
	_w_gyro_bias(this, "W_GYRO_BIAS"),
	_mag_decl(this, "MAG_DECL"),
	_mag_decl_a(this, "MAG_DECL_A"),
	_ext_hdg_m(this, "EXT_HDG_M"),
	_acc_comp(this, "ACC_COMP"),
	_bias_max(this, "BIAS_MAX"),
	_vibe_thresh(this, "VIBE_THRESH"),
	_lp_roll_rate(this, "PQ_LP"),
	_lp_pitch_rate(this, "PQ_LP"),
	_lp_yaw_rate(this, "R_LP")
{
}

void BlockAttEstQ::update()
{
}

void BlockAttEstQ::correctMagAccel(const Vec6 & y, const SMat6 & R)
{
}

void BlockAttEstQ::predict(const Vec6 & y, const SMat6 & R)
{
}
