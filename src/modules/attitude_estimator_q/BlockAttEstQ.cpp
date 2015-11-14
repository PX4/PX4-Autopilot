#include "BlockAttEstQ.hpp"
#include <matrix/integration.hpp>

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
	_lp_yaw_rate(this, "R_LP"),
	_q_nb(1, 0, 0, 0),
	_b(0, 0, 0)
{
}

void BlockAttEstQ::correctMagAccel(const Vec6 & y, const SMat6 & R)
{
	Dcmf C_nb(_q_nb);
	Dcmf C_bn = C_nb.T();
	Vec3 down_b = C_bn*Vec3(0, 0, 1);
}

void BlockAttEstQ::predict(const Vec6 & y, const SMat6 & R)
{
	Vec3 omega_nb(y(0), y(1), y(2));
	integrate_rk4(&f_dynamics,
		_q_nb, omega_nb, 0.0f, getDt(), _q_nb);
}

Vec4 BlockAttEstQ::f_dynamics(float t,
		const Matrix<float, 4, 1> &q_nb,
		const Matrix<float, 3, 1> &omega_nb)
{
	return Quatf(q_nb).derivative(omega_nb);
}


