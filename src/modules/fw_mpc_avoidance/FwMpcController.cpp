#include "FwMpcController.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

#include <algorithm>
#include <cmath>
#include <vector>

using matrix::Matrix;
using matrix::Vector3f;
using matrix::Vector4f;

namespace
{
template<typename Mat>
void denseToCSC(const Mat &M, int rows, int cols, bool upper_only,
		std::vector<OSQPFloat> &data, std::vector<OSQPInt> &indices, std::vector<OSQPInt> &indptr)
{
	data.clear();
	indices.clear();
	indptr.clear();
	indptr.reserve(cols + 1);

	const float eps = 1e-6f;

	for (int j = 0; j < cols; j++) {
		indptr.push_back((OSQPInt)data.size());

		for (int i = 0; i < rows; i++) {
			if (upper_only && i > j) {
				continue;
			}

			const float v = M(i, j);

			if (fabsf(v) > eps) {
				data.push_back(v);
				indices.push_back((OSQPInt)i);
			}
		}
	}

	indptr.push_back((OSQPInt)data.size());
}
} // namespace

bool FwMpcController::configure(float Ts, int horizon)
{
	if (horizon < 1 || horizon > kMaxHorizon) {
		PX4_ERR("MPC horizon %d exceeds limit %d", horizon, kMaxHorizon);
		return false;
	}

	_Ts = Ts;
	_N = horizon;
	_ubar.setZero();
	_xbar.setZero();
	_last_qp_status = 0;
	return true;
}

void FwMpcController::set_obstacles(const std::vector<Obstacle> &obs)
{
	_n_obstacles = std::min((int)obs.size(), kMaxObstacles);

	for (int i = 0; i < _n_obstacles; i++) {
		_obstacles[i] = obs[i];
	}
}

void FwMpcController::clear_obstacles()
{
	_n_obstacles = 0;
}

FwMpcController::StateVec FwMpcController::initTrim(float V_trim, float z0_up, const matrix::Vector3f &goal_up)
{
	_V_trim = V_trim;

	const float mass = _model.mass();
	const float qdyn = 0.5f * _model.rho() * V_trim * V_trim * _model.wing_area();
	const float CLreq = mass * _model.gravity() / qdyn;

	_alpha_trim = (CLreq - _model.CL0()) / _model.CL_alpha();
	const float CD = _model.CD0() + _model.induced_drag_k() * CLreq * CLreq;
	_T_trim = 0.5f * _model.rho() * V_trim * V_trim * _model.wing_area() * CD;
	_theta_trim = _alpha_trim;

	const float u0 = V_trim * cosf(_alpha_trim);
	const float v0 = 0.f;
	const float w0 = V_trim * sinf(_alpha_trim);

	const Vector3f dp = goal_up - Vector3f{0.f, 0.f, z0_up};
	const float psi0 = atan2f(dp(1), dp(0));

	StateVec x0{};
	x0(0) = u0;
	x0(1) = v0;
	x0(2) = w0;
	x0(3) = 0.f;
	x0(4) = 0.f;
	x0(5) = 0.f;
	x0(6) = 0.f;
	x0(7) = _theta_trim;
	x0(8) = psi0;
	x0(9) = 0.f;
	x0(10) = 0.f;
	x0(11) = z0_up;

	const ControlVec u_trim{0.f, 0.f, 0.f, _T_trim};

	for (int k = 0; k < _N; k++) {
		_ubar.col(k) = u_trim;
	}

	_xbar.col(0) = x0;

	for (int k = 0; k < _N; k++) {
		const StateVec xk = _xbar.col(k);
		const ControlVec uk = _ubar.col(k);
		_xbar.col(k + 1) = fd_step(xk, uk);
	}

	return x0;
}

void FwMpcController::set_vehicle_params(float mass, const matrix::Vector3f &inertia_diag, float kdv, float kdw)
{
	_model.set_mass(mass);
	_model.set_inertia_diag(inertia_diag);
	_model.set_damping(kdv, kdw);
}

bool FwMpcController::step(const StateVec &x_now, const matrix::Vector3f &goal_up, float V_cruise, bool is_last,
			   ControlVec &u_apply, StateVec &x_next)
{
	(void)is_last; // currently unused

	matrix::Matrix<float, 3, kMaxHorizon> x_ref_seq{};

	for (int k = 0; k < _N; k++) {
		x_ref_seq(0, k) = goal_up(0);
		x_ref_seq(1, k) = goal_up(1);
		x_ref_seq(2, k) = goal_up(2);
	}

	const int nIt = (_options.mode == Mode::LMPC) ? 1 : 2;
	bool solved = false;

	for (int it = 0; it < nIt; it++) {
		_xbar.col(0) = x_now;

		for (int k = 0; k < _N; k++) {
			const StateVec xk = _xbar.col(k);
			const ControlVec uk = _ubar.col(k);
			_xbar.col(k + 1) = fd_step(xk, uk);
		}

		matrix::Vector<float, kMaxHorizon> theta_ref_seq{};
		matrix::Vector<float, kMaxHorizon> T_ref_seq{};

		if (_options.recompute_ff) {
			ff_refs_from_nominal(_xbar, x_ref_seq, V_cruise, theta_ref_seq, T_ref_seq);

		} else {
			for (int k = 0; k < _N; k++) {
				theta_ref_seq(k) = _theta_trim;
				T_ref_seq(k) = _T_trim;
			}
		}

		matrix::Vector<float, kMaxHorizon> psi_ref_seq{};
		heading_refs(_xbar, x_ref_seq, psi_ref_seq);

		std::array<StateMat, kMaxHorizon> Ak{};
		std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> Bk{};

		for (int k = 0; k < _N; k++) {
			const StateVec xk = _xbar.col(k);
			const ControlVec uk = _ubar.col(k);
			lin_fd(xk, uk, Ak[k], Bk[k]);
		}

		matrix::Vector<float, kMaxVars> z{};
		int n_vars = 0;
		int n_constraints = 0;

		if (buildQP(_xbar, _ubar, x_ref_seq, theta_ref_seq, T_ref_seq, psi_ref_seq, Ak, Bk, _N, n_vars,
			    n_constraints)) {
			solved = solveQP(z, n_vars, n_constraints);
		}

		const int Nz_dx = _N * kStateSize;

		for (int k = 0; k < _N; k++) {
			ControlVec du{};

			for (int i = 0; i < kControlSize; i++) {
				du(i) = z(Nz_dx + k * kControlSize + i);
			}

			ControlVec u_new = _ubar.col(k) + du;

			for (int i = 0; i < kControlSize; i++) {
				u_new(i) = math::constrain(u_new(i), _limits.u_min(i), _limits.u_max(i));
			}

			_ubar.col(k) = u_new;
		}
	}

	u_apply = _ubar.col(0);

	for (int i = 0; i < kControlSize; i++) {
		u_apply(i) = math::constrain(u_apply(i), _limits.u_min(i), _limits.u_max(i));
	}

	x_next = fd_step(x_now, u_apply);

	// Shift horizon
	for (int k = 0; k < _N - 1; k++) {
		_ubar.col(k) = _ubar.col(k + 1);
	}

	_ubar.col(_N - 1) = _ubar.col(_N - 2);

	_xbar.col(0) = x_next;

	for (int k = 0; k < _N; k++) {
		const StateVec xk = _xbar.col(k);
		const ControlVec uk = _ubar.col(k);
		_xbar.col(k + 1) = fd_step(xk, uk);
	}

	return solved;
}

FwMpcController::StateVec FwMpcController::fd_step(const StateVec &x0, const ControlVec &u) const
{
	return _model.rk4_step(x0, u, _Ts);
}

void FwMpcController::lin_fd(const StateVec &x, const ControlVec &u, StateMat &A,
			     matrix::Matrix<float, kStateSize, kControlSize> &B) const
{
	const StateVec f0 = fd_step(x, u);
	const float epsx = 1e-4f;
	const float epsu = 1e-4f;

	for (int i = 0; i < kStateSize; i++) {
		StateVec dx = x;
		dx(i) += epsx;
		const StateVec f_plus = fd_step(dx, u);

		for (int r = 0; r < kStateSize; r++) {
			A(r, i) = (f_plus(r) - f0(r)) / epsx;
		}
	}

	for (int j = 0; j < kControlSize; j++) {
		ControlVec du = u;
		du(j) += epsu;
		const StateVec f_plus = fd_step(x, du);

		for (int r = 0; r < kStateSize; r++) {
			B(r, j) = (f_plus(r) - f0(r)) / epsu;
		}
	}
}

void FwMpcController::ff_refs_from_nominal(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, float Vc,
		matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
		matrix::Vector<float, kMaxHorizon> &T_ref_seq) const
{
	const float mass = _model.mass();
	const float rho = _model.rho();
	const float S = _model.wing_area();
	const float g = _model.gravity();

	for (int k = 0; k < _N; k++) {
		const Vector3f vel{xbar(0, k), xbar(1, k), xbar(2, k)};
		const float V = math::max(vel.norm(), 1.f);
		const float phi = xbar(6, k);
		const float z_now = xbar(11, k);
		const float z_targ = x_ref_seq(2, k);

		float vz_des = (z_targ - z_now) / _Ts;
		vz_des = math::constrain(vz_des, -0.2f * Vc, 0.2f * Vc);

		const float gamma_ref = asinf(math::constrain(vz_des / math::max(V, 1e-3f), -0.25f, 0.25f));
		const float qd = 0.5f * rho * V * V;
		const float Lreq = mass * g * cosf(gamma_ref) / math::max(cosf(phi), 0.2f);
		const float CLreq = Lreq / (qd * S);
		const float alpha_ref = (CLreq - _model.CL0()) / _model.CL_alpha();
		const float theta_ref = alpha_ref + gamma_ref;
		const float CD_ref = _model.CD0() + _model.induced_drag_k() * (CLreq * CLreq);
		const float T_ref = qd * S * CD_ref + mass * g * sinf(gamma_ref);

		theta_ref_seq(k) = theta_ref;
		T_ref_seq(k) = T_ref;
	}
}

void FwMpcController::heading_refs(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				   const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
				   matrix::Vector<float, kMaxHorizon> &psi_ref) const
{
	for (int k = 0; k < _N; k++) {
		const Vector3f pos{xbar(9, k), xbar(10, k), xbar(11, k)};
		const Vector3f dp = Vector3f{x_ref_seq(0, k), x_ref_seq(1, k), x_ref_seq(2, k)} - pos;
		psi_ref(k) = atan2f(dp(1), dp(0));
	}
}

float FwMpcController::angDiff(float a, float b) const
{
	return matrix::wrap_pi(a - b);
}

bool FwMpcController::buildQP(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			      const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
			      const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			      const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
			      const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
			      const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
			      const std::array<StateMat, kMaxHorizon> &Ak,
			      const std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk,
			      int N, int &n_vars, int &n_constraints)
{
	const int n = kStateSize;
	const int m = kControlSize;
	const int Nz_dx = N * n;
	const int Nz_du = N * m;
	n_vars = Nz_dx + Nz_du;

	_H.setZero();
	_f.setZero();
	_A.setZero();

	for (int i = 0; i < kMaxConstraints; i++) {
		_l(i) = -OSQP_INFTY;
		_u(i) = OSQP_INFTY;
	}

	// Equality constraints for linearized dynamics
	int row = 0;

	for (int k = 0; k < N; k++) {
		const int idx_dxk = k * n;

		for (int i = 0; i < n; i++) {
			_A(row + i, idx_dxk + i) = 1.f;
			_l(row + i) = 0.f;
			_u(row + i) = 0.f;
		}

		if (k == 0) {
			const int idx_du0 = Nz_dx;

			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					_A(row + i, idx_du0 + j) = -Bk[0](i, j);
				}
			}

		} else {
			const int idx_dxkm1 = (k - 1) * n;
			const int idx_dukm1 = Nz_dx + (k - 1) * m;

			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					_A(row + i, idx_dxkm1 + j) = -Ak[k - 1](i, j);
				}

				for (int j = 0; j < m; j++) {
					_A(row + i, idx_dukm1 + j) = -Bk[k - 1](i, j);
				}
			}
		}

		row += n;
	}

	int row_offset = row;

	// Selectors
	Matrix<float, 3, n> Spos{};
	Spos(0, 9) = 1.f;
	Spos(1, 10) = 1.f;
	Spos(2, 11) = 1.f;
	Matrix<float, 3, n> Sang{};
	Sang(0, 6) = 1.f;
	Sang(1, 7) = 1.f;
	Sang(2, 8) = 1.f;

	const Matrix<float, n, 3> Spos_T = Spos.transpose();
	const Matrix<float, n, 3> Sang_T = Sang.transpose();

	Matrix<float, n, n> epsI{};
	epsI.setZero();

	for (int i = 0; i < n; i++) {
		epsI(i, i) = 2.f * 1e-5f;
	}

	const Matrix<float, n, n> Qpos = 2.f * (Spos_T * (_weights.Qp * Spos));
	const Matrix<float, n, n> Qang = 2.f * (Sang_T * (_weights.Qang * Sang));

	// Stage costs
	for (int k = 0; k < N; k++) {
		const int idx_dxk = k * n;
		const int idx_duk = Nz_dx + k * m;

		const StateVec xk = xbar.col(k);
		const Vector3f ref_k{x_ref_seq(0, k), x_ref_seq(1, k), x_ref_seq(2, k)};
		const Vector3f epos = (Spos * xk) - ref_k;
		const Vector3f eang{xk(6), xk(7) - theta_ref_seq(k), angDiff(xk(8), psi_ref_seq(k))};

		const Matrix<float, n, n> Hdx = Qpos + Qang + epsI;
		const matrix::Vector<float, kStateSize> fdx = 2.f * (Spos_T * (_weights.Qp * epos) + Sang_T * (_weights.Qang * eang));

		for (int i = 0; i < n; i++) {
			_f(idx_dxk + i) += fdx(i);

			for (int j = 0; j < n; j++) {
				_H(idx_dxk + i, idx_dxk + j) += Hdx(i, j);
			}
		}

		const Matrix<float, m, m> Hdu = 2.f * (_weights.Rdu + _weights.Ru_abs);

		for (int i = 0; i < m; i++) {
			for (int j = 0; j < m; j++) {
				_H(idx_duk + i, idx_duk + j) += Hdu(i, j);
			}
		}

		const ControlVec u_ref{0.f, 0.f, 0.f, T_ref_seq(k)};
		const ControlVec fdu = 2.f * (_weights.Ru_abs * (ubar.col(k) - u_ref));

		for (int i = 0; i < m; i++) {
			_f(idx_duk + i) += fdu(i);
		}
	}

	// Terminal position cost on last dx block
	{
		const int idx_dxN = (N - 1) * n;
		const StateVec xN = xbar.col(N - 1);
		const Vector3f ref_N{x_ref_seq(0, N - 1), x_ref_seq(1, N - 1), x_ref_seq(2, N - 1)};
		const Vector3f eN = (Spos * xN) - ref_N;
		const Matrix<float, n, n> Hterm = 2.f * _weights.Qterm * (Spos_T * (_weights.Qp * Spos));
		const matrix::Vector<float, kStateSize> fterm = 2.f * _weights.Qterm * (Spos_T * (_weights.Qp * eN));

		for (int i = 0; i < n; i++) {
			_f(idx_dxN + i) += fterm(i);

			for (int j = 0; j < n; j++) {
				_H(idx_dxN + i, idx_dxN + j) += Hterm(i, j);
			}
		}
	}

	// Smoothness along horizon
	if (_limits.use_stage_smoothness) {
		for (int k = 0; k < N - 1; k++) {
			const int idx_du = Nz_dx + k * m;
			const int idx_du_next = Nz_dx + (k + 1) * m;

			for (int i = 0; i < m; i++) {
				const float w = _weights.Rrate_diag(i);

				if (w <= 0.f) {
					continue;
				}

				const float dubar = ubar(i, k + 1) - ubar(i, k);
				const float two_w = 2.f * w;

				_H(idx_du + i, idx_du + i) += two_w;
				_H(idx_du_next + i, idx_du_next + i) += two_w;
				_H(idx_du + i, idx_du_next + i) -= two_w;
				_H(idx_du_next + i, idx_du + i) -= two_w;

				_f(idx_du + i) += -two_w * dubar;
				_f(idx_du_next + i) += two_w * dubar;
			}
		}
	}

	addObstacleConstraints(xbar, x_ref_seq, N, row_offset, Nz_dx);

	if (_limits.use_stage_smoothness && _limits.use_rate_limits) {
		addRateConstraints(ubar, N, row_offset, Nz_dx, Nz_du);
	}

	addBounds(ubar, N, row_offset, Nz_dx, Nz_du);

	if (row_offset > kMaxConstraints) {
		PX4_ERR("QP constraint buffer overflow (%d > %d)", row_offset, kMaxConstraints);
		return false;
	}

	n_constraints = row_offset;
	return true;
}

bool FwMpcController::solveQP(matrix::Vector<float, kMaxVars> &z, int n_vars, int n_constraints)
{
	// Symmetrize H
	for (int i = 0; i < n_vars; i++) {
		for (int j = i + 1; j < n_vars; j++) {
			const float v = 0.5f * (_H(i, j) + _H(j, i));
			_H(i, j) = v;
			_H(j, i) = v;
		}
	}

	std::vector<OSQPFloat> P_data;
	std::vector<OSQPInt> P_i;
	std::vector<OSQPInt> P_p;
	denseToCSC(_H, n_vars, n_vars, true, P_data, P_i, P_p);

	std::vector<OSQPFloat> A_data;
	std::vector<OSQPInt> A_i;
	std::vector<OSQPInt> A_p;
	denseToCSC(_A, n_constraints, n_vars, false, A_data, A_i, A_p);

	std::vector<OSQPFloat> q_vec(n_vars, 0.f);

	for (int i = 0; i < n_vars; i++) {
		q_vec[i] = _f(i);
	}

	std::vector<OSQPFloat> l_vec(n_constraints, -OSQP_INFTY);
	std::vector<OSQPFloat> u_vec(n_constraints, OSQP_INFTY);

	for (int i = 0; i < n_constraints; i++) {
		l_vec[i] = _l(i);
		u_vec[i] = _u(i);
	}

	OSQPCscMatrix *P = OSQPCscMatrix_new(n_vars, n_vars, (OSQPInt)P_data.size(), P_data.data(), P_i.data(),
					     P_p.data());
	OSQPCscMatrix *A = OSQPCscMatrix_new(n_constraints, n_vars, (OSQPInt)A_data.size(), A_data.data(), A_i.data(),
					     A_p.data());
	OSQPSettings *settings = OSQPSettings_new();

	if (!P || !A || !settings) {
		OSQPCscMatrix_free(A);
		OSQPCscMatrix_free(P);
		OSQPSettings_free(settings);
		_last_qp_status = -1;
		z.setZero();
		return false;
	}

	osqp_set_default_settings(settings);
	settings->verbose = 0;
	settings->alpha = 1.2f;
	settings->eps_abs = 1e-3f;
	settings->eps_rel = 1e-3f;
	settings->warm_starting = 1;
	settings->max_iter = 400;

	OSQPSolver *solver = nullptr;
	OSQPInt exitflag = osqp_setup(&solver, P, q_vec.data(), A, l_vec.data(), u_vec.data(), n_constraints, n_vars,
				      settings);

	if (exitflag == 0) {
		exitflag = osqp_solve(solver);
	}

	bool ok = false;

	if (solver && solver->info) {
		_last_qp_status = solver->info->status_val;

		ok = (solver->info->status_val == OSQP_SOLVED)
		     || (solver->info->status_val == OSQP_SOLVED_INACCURATE);

	} else {
		_last_qp_status = exitflag;
	}

	if (ok && solver && solver->solution && solver->solution->x) {
		for (int i = 0; i < n_vars; i++) {
			z(i) = solver->solution->x[i];
		}

	} else {
		z.setZero();
	}

	osqp_cleanup(solver);
	OSQPCscMatrix_free(A);
	OSQPCscMatrix_free(P);
	OSQPSettings_free(settings);
	return ok;
}

void FwMpcController::addObstacleConstraints(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, int N, int &row_offset, int Nz_dx)
{
	(void)x_ref_seq;

	for (int k = 0; k < N; k++) {
		const Vector3f pbar{xbar(9, k), xbar(10, k), xbar(11, k)};
		const int idx_dxk = k * kStateSize;

		for (int j = 0; j < _n_obstacles; j++) {
			const float Rbuf = _obstacles[j].R + _obstacles[j].margin;
			Vector3f dvec = pbar - _obstacles[j].c;
			float d = dvec.norm();

			if (d < 1e-6f) {
				d = 1e-6f;
				dvec = Vector3f{1.f, 0.f, 0.f};
			}

			const float gbar = Rbuf - d;
			const Vector3f gradg = -(dvec / d);

			if (row_offset >= kMaxConstraints) {
				return;
			}

			_A(row_offset, idx_dxk + 9) = gradg(0);
			_A(row_offset, idx_dxk + 10) = gradg(1);
			_A(row_offset, idx_dxk + 11) = gradg(2);
			_u(row_offset) = -gbar;
			_l(row_offset) = -OSQP_INFTY;
			row_offset++;
		}
	}
}

void FwMpcController::addRateConstraints(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
		int &row_offset, int Nz_dx, int Nz_du)
{
	(void)Nz_du;

	for (int k = 0; k < N - 1; k++) {
		const int idx_du = Nz_dx + k * kControlSize;
		const int idx_du_next = Nz_dx + (k + 1) * kControlSize;

		for (int i = 0; i < kControlSize; i++) {
			const float dubar = ubar(i, k + 1) - ubar(i, k);
			const float limit = _limits.du_rate(i);

			if (row_offset < kMaxConstraints) {
				_A(row_offset, idx_du_next + i) = 1.f;
				_A(row_offset, idx_du + i) = -1.f;
				_u(row_offset) = limit - dubar;
				_l(row_offset) = -OSQP_INFTY;
				row_offset++;
			}

			if (row_offset < kMaxConstraints) {
				_A(row_offset, idx_du_next + i) = -1.f;
				_A(row_offset, idx_du + i) = 1.f;
				_u(row_offset) = limit + dubar;
				_l(row_offset) = -OSQP_INFTY;
				row_offset++;
			}
		}
	}
}

void FwMpcController::addBounds(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N, int &row_offset,
				int Nz_dx, int Nz_du)
{
	(void)Nz_du;

	for (int k = 0; k < N; k++) {
		const int idx_duk = Nz_dx + k * kControlSize;

		for (int i = 0; i < kControlSize; i++) {
			if (row_offset >= kMaxConstraints) {
				return;
			}

			_A(row_offset, idx_duk + i) = 1.f;
			_l(row_offset) = _limits.u_min(i) - ubar(i, k);
			_u(row_offset) = _limits.u_max(i) - ubar(i, k);
			row_offset++;
		}
	}
}
