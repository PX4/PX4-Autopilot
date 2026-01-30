#pragma once

#include "FixedWingMpcModel.hpp"

#include <matrix/matrix/math.hpp>
#include <osqp.h>

#include <array>
#include <vector>

/**
 * Fixed-wing MPC controller translated from the UAVMPC_LOW MATLAB prototype.
 * Builds a horizon-based QP (linearized dynamics, smoothness, obstacle constraints)
 * and solves it with OSQP to generate control increments.
 */
class FwMpcController
{
public:
	static constexpr int kStateSize = FixedWingMpcModel::kStateSize;   // 12
	static constexpr int kControlSize = FixedWingMpcModel::kControlSize; // 4
	static constexpr int kMaxHorizon = 24;
	static constexpr int kMaxVars = kMaxHorizon * (kStateSize + kControlSize); // dx (N*n) + du (N*m)
	static constexpr int kMaxConstraints = 1200; // N*n eq + obstacle/rate/bounds
	static constexpr int kMaxObstacles = 4;

	using StateVec = FixedWingMpcModel::State;
	using ControlVec = FixedWingMpcModel::Control;

	enum class Mode {
		LMPC,   // 1 RTI iteration
		SLMPC   // 2 RTI iterations
	};

	struct Weights {
		matrix::Matrix3f Qp{matrix::diag(matrix::Vector3f{4.f, 4.f, 6.f})};
		float Qterm{20.f};
		matrix::Matrix3f Qang{matrix::diag(matrix::Vector3f{0.25f, 0.60f, 0.80f})};
		matrix::SquareMatrix<float, kControlSize> Rdu{matrix::diag(matrix::Vector4f{1.f, 1.f, 1.f, 0.15f})};
		matrix::SquareMatrix<float, kControlSize> Ru_abs{matrix::diag(matrix::Vector4f{0.03f, 0.03f, 0.03f, 0.03f})};
		matrix::Vector4f Rrate_diag{0.5f, 0.5f, 0.5f, 0.10f};
	};

	struct Limits {
		matrix::Vector4f u_min{-0.1f, -0.2f, -0.1f, 0.f};
		matrix::Vector4f u_max{0.1f, 0.3f, 0.1f, 24.f};
		bool use_stage_smoothness{true};
		bool use_rate_limits{false};
		matrix::Vector4f du_rate{0.15f, 0.15f, 0.15f, 0.30f};
	};

	struct Options {
		Mode mode{Mode::SLMPC};
		bool recompute_ff{true};
	};

	struct Obstacle {
		matrix::Vector3f c{0.f, 0.f, 0.f};
		float R{0.f};
		float margin{0.f};
	};

	FwMpcController() = default;

	bool configure(float Ts, int horizon);

	void set_obstacles(const std::vector<Obstacle> &obs);
	void clear_obstacles();

	StateVec initTrim(float V_trim, float z0_up, const matrix::Vector3f &goal_up);

	/**
	 * One MPC receding-horizon step.
	 * @param x_now current state (z up, same convention as FixedWingMpcModel)
	 * @param goal_up desired waypoint in inertial frame (z up)
	 * @param V_cruise reference cruise speed
	 * @param is_last whether this is the final waypoint (not used yet)
	 * @param u_apply control to apply (du integrated on top of nominal)
	 * @param x_next nominal state after applying u_apply for Ts
	 * @return true if QP solved, false if fallback zero solution used
	 */
	bool step(const StateVec &x_now, const matrix::Vector3f &goal_up, float V_cruise, bool is_last,
		  ControlVec &u_apply, StateVec &x_next);

	int last_qp_status() const { return _last_qp_status; }

	Weights &weights() { return _weights; }
	Limits &limits() { return _limits; }
	const Limits &limits() const { return _limits; }
	Options &options() { return _options; }

	const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar() const { return _ubar; }

	// Tune vehicle dynamics (shared with SIH parameters)
	void set_vehicle_params(float mass, const matrix::Vector3f &inertia_diag, float kdv, float kdw);

private:
	using StateMat = matrix::SquareMatrix<float, kStateSize>;

	StateVec fd_step(const StateVec &x0, const ControlVec &u) const;
	void lin_fd(const StateVec &x, const ControlVec &u, StateMat &A, matrix::Matrix<float, kStateSize, kControlSize> &B) const;
	void ff_refs_from_nominal(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				  const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, float Vc,
				  matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
				  matrix::Vector<float, kMaxHorizon> &T_ref_seq) const;
	void heading_refs(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			  const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			  matrix::Vector<float, kMaxHorizon> &psi_ref) const;
	float angDiff(float a, float b) const;

	bool buildQP(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		     const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
		     const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
		     const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
		     const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
		     const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
		     const std::array<StateMat, kMaxHorizon> &Ak,
		     const std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk,
		     int N, int &n_vars, int &n_constraints);
	bool solveQP(matrix::Vector<float, kMaxVars> &z, int n_vars, int n_constraints);

	void addObstacleConstraints(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				    const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
				    int N, int &row_offset, int Nz_dx);
	void addRateConstraints(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
				int &row_offset, int Nz_dx, int Nz_du);
	void addBounds(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
		       int &row_offset, int Nz_dx, int Nz_du);

	FixedWingMpcModel _model{};

	float _Ts{0.1f};
	int _N{10};

	Weights _weights{};
	Limits _limits{};
	Options _options{};

	float _V_trim{13.f};
	float _alpha_trim{0.f};
	float _theta_trim{0.f};
	float _T_trim{0.f};

	matrix::Matrix<float, kStateSize, kMaxHorizon + 1> _xbar{};
	matrix::Matrix<float, kControlSize, kMaxHorizon> _ubar{};

	std::array<Obstacle, kMaxObstacles> _obstacles{};
	int _n_obstacles{0};

	// QP storage
	matrix::SquareMatrix<float, kMaxVars> _H{};
	matrix::Vector<float, kMaxVars> _f{};
	matrix::Matrix<float, kMaxConstraints, kMaxVars> _A{};
	matrix::Vector<float, kMaxConstraints> _l{};
	matrix::Vector<float, kMaxConstraints> _u{};
	int _last_qp_status{0};
};
