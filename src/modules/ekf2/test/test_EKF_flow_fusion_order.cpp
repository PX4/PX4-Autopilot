/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Sequential fusion of a vector observation must equal the vector (batch) Kalman update at the
 * linearisation point. The batch update has no processing order, so this is the order invariance of
 * NASA/TP-2018-219822 Algorithm 4.1 stated as an equality, checked on the optical flow fusion (two
 * coupled scalar observations) and on the generic measurementUpdate() helper.
 */

#include <cfloat>
#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "../EKF/python/ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h"
#include "../EKF/python/ekf_derivation/generated/compute_flow_y_innov_var_and_h.h"

using namespace matrix;

class EkfFlowFusionOrderTest : public ::testing::Test
{
public:
	using VectorState = Ekf::VectorState;
	using SquareMatrixState = Ekf::SquareMatrixState;
	using Vector25 = Vector<float, 25>;

	std::shared_ptr<Ekf> _ekf{std::make_shared<Ekf>()};

	StateSample _x0{};
	LatLonAlt _gpos0{0.0, 0.0, 0.f};
	SquareMatrixState _P0{};

	void SetUp() override
	{
		_ekf->init(0);
		_ekf->_control_status.flags.in_air = true;
		_ekf->_control_status.flags.heading_observable = true;
		_ekf->_control_status.flags.mag = false;
		_ekf->_time_delayed_us = 1'000'000;
		_ekf->_params.ekf2_of_gate = 3.f;
		_ekf->_flow_gyro_bias.setZero();
		_ekf->_flow_sample_delayed.gyro_rate.setZero();
		_ekf->_flow_sample_delayed.quality = 255;
	}

	// a priori state and covariance: uncertain velocity, attitude and terrain with cross-correlations;
	// the magnetometer and wind states are left uncorrelated so that their inhibited gains are zero anyway
	void setPrior(const Vector3f &vel, float hagl, float velocity_correlation)
	{
		_x0 = StateSample{};
		_x0.quat_nominal = Quatf(Eulerf(0.05f, -0.03f, 0.7f));
		_x0.vel = vel;
		const float altitude = 10.f;
		_gpos0 = LatLonAlt(47.0, 8.0, altitude);
		_x0.pos = Vector3f(0.f, 0.f, -altitude);
		_x0.terrain = hagl - altitude; // getHagl() = terrain + altitude
		_x0.mag_I = Vector3f(0.2f, 0.f, 0.4f);

		_P0.setZero();
		const float sig_att = 0.02f, sig_vel = 0.5f, sig_pos = 1.f, sig_gb = 1e-3f, sig_ab = 0.05f;
		const float sig_mag = 1e-3f, sig_wind = 0.5f, sig_ter = 0.5f;

		for (int i = 0; i < 3; i++) {
			_P0(State::quat_nominal.idx + i, State::quat_nominal.idx + i) = sig_att * sig_att;
			_P0(State::vel.idx + i, State::vel.idx + i) = sig_vel * sig_vel;
			_P0(State::pos.idx + i, State::pos.idx + i) = sig_pos * sig_pos;
			_P0(State::gyro_bias.idx + i, State::gyro_bias.idx + i) = sig_gb * sig_gb;
			_P0(State::accel_bias.idx + i, State::accel_bias.idx + i) = sig_ab * sig_ab;
			_P0(State::mag_I.idx + i, State::mag_I.idx + i) = sig_mag * sig_mag;
			_P0(State::mag_B.idx + i, State::mag_B.idx + i) = sig_mag * sig_mag;
		}

		for (int i = 0; i < 2; i++) {
			_P0(State::wind_vel.idx + i, State::wind_vel.idx + i) = sig_wind * sig_wind;
		}

		_P0(State::terrain.idx, State::terrain.idx) = sig_ter * sig_ter;

		// the two flow components observe different velocity components; couple them through the velocity
		// covariance, and couple both with the attitude and the terrain
		symmetric(State::vel.idx, State::vel.idx + 1, velocity_correlation * sig_vel * sig_vel);
		symmetric(State::vel.idx, State::quat_nominal.idx + 1, 0.3f * sig_vel * sig_att);
		symmetric(State::vel.idx + 1, State::quat_nominal.idx, -0.3f * sig_vel * sig_att);
		symmetric(State::vel.idx, State::terrain.idx, 0.4f * sig_vel * sig_ter);
		symmetric(State::vel.idx + 1, State::terrain.idx, -0.2f * sig_vel * sig_ter);
		symmetric(State::vel.idx, State::accel_bias.idx, 0.5f * sig_vel * sig_ab);
		symmetric(State::vel.idx + 1, State::accel_bias.idx + 1, 0.5f * sig_vel * sig_ab);

		restorePrior();
	}

	void symmetric(int i, int j, float value) { _P0(i, j) = value; _P0(j, i) = value; }

	void restorePrior()
	{
		_ekf->_state = _x0;
		_ekf->_gpos = _gpos0;
		_ekf->_R_to_earth = Dcmf(_x0.quat_nominal);
		_ekf->P = _P0;
	}

	const StateSample &state() const { return _ekf->_state; }
	const SquareMatrixState &covariance() const { return _ekf->P; }
	const estimator_aid_source2d_s &flowAidSrc() const { return _ekf->_aid_src_optical_flow; }

	// observation jacobians and innovation variances of both flow components at the prior
	void lineariseFlow(float R, VectorState &Hx, VectorState &Hy, Vector2f &innov_var)
	{
		// same regularisation as controlOpticalFlowFusion() and fuseOptFlow() use
		const float epsilon = 1e-3f;
		Vector2f iv;
		sym::ComputeFlowXyInnovVarAndHx(_x0.vector(), _P0, R, epsilon, &iv, &Hx);
		float iv_y;
		sym::ComputeFlowYInnovVarAndH(_x0.vector(), _P0, R, epsilon, &iv_y, &Hy);
		innov_var = Vector2f(iv(0), iv_y);
	}

	// vector Kalman update of k scalar observations at the prior, Joseph form, PX4 sign convention
	// (state -= K * innovation) and the same gain inhibits as the sequential fusion
	template<int k>
	void batchUpdate(const Matrix<float, k, State::size> &H, const Vector<float, k> &R, const Vector<float, k> &innov,
			 StateSample &x_out, SquareMatrixState &P_out)
	{
		const SquareMatrix<float, k> S = H * _P0 * H.transpose() + diag(R);
		Matrix<float, State::size, k> K = _P0 * H.transpose() * inv(S);

		for (int j = 0; j < k; j++) {
			VectorState col = K.col(j);
			_ekf->clearInhibitedStateKalmanGains(col);
			K.col(j) = col;
		}

		const SquareMatrixState A = eye<float, State::size>() - K * H;
		P_out = A * _P0 * A.transpose() + K * diag(R) * K.transpose();

		VectorState dx;

		for (int j = 0; j < k; j++) {
			dx -= VectorState(K.col(j)) * innov(j);
		}

		restorePrior();
		_ekf->applyStateCorrection(dx);
		x_out = _ekf->_state;
	}

	// the sequential optical flow fusion under test, fed like controlOpticalFlowFusion() does
	bool sequentialFlowUpdate(const VectorState &Hx, const Vector2f &innov_var, float R, const Vector2f &innov)
	{
		restorePrior();
		// PX4 convention: innovation = prediction - observation
		const Vector2f observation = _ekf->predictFlow(_ekf->_flow_sample_delayed.gyro_rate) - innov;
		_ekf->updateAidSourceStatus(_ekf->_aid_src_optical_flow, _ekf->_time_delayed_us, observation, Vector2f(R, R),
					    innov, innov_var, _ekf->_params.ekf2_of_gate);
		VectorState H = Hx;
		return _ekf->fuseOptFlow(H, true);
	}

	// generic sequential fusion of two scalar observations through the helpers, in the given order
	void sequentialHelperUpdate(const VectorState h[2], const float R[2], const float innov[2], const int order[2])
	{
		restorePrior();
		VectorState state_correction;

		for (int n = 0; n < 2; n++) {
			const int j = order[n];
			const VectorState PH = _ekf->P * h[j];
			const float innov_var = PH.dot(h[j]) + R[j];
			VectorState K = PH / innov_var;
			_ekf->measurementUpdate(K, h[j], R[j], innov[j], state_correction);
		}

		_ekf->applyStateCorrection(state_correction);
	}

	static float maxAbsDiff(const SquareMatrixState &a, const SquareMatrixState &b)
	{
		float m = 0.f;

		for (int i = 0; i < State::size; i++) {
			for (int j = 0; j < State::size; j++) {
				m = fmaxf(m, fabsf(a(i, j) - b(i, j)));
			}
		}

		return m;
	}
};

TEST_F(EkfFlowFusionOrderTest, flowFusionEqualsBatchUpdate)
{
	// low and fast over the ground: the flow observation is strongly nonlinear in the height and both
	// components carry a large innovation inside the gate
	setPrior(Vector3f(1.5f, -0.5f, 0.f), 2.f, 0.6f);
	const float R = 0.05f * 0.05f;

	VectorState Hx, Hy;
	Vector2f innov_var;
	lineariseFlow(R, Hx, Hy, innov_var);

	// 2.5 sigma on both axes: inside the 3 sigma gate a priori. The sign of the second axis is chosen so
	// that fusing the first axis first pushes the second innovation further out when it is recomputed
	// from the corrected state (the failure mode of section 4.2 of the whitepaper).
	Matrix<float, 2, State::size> H;
	H.row(0) = Hx.transpose();
	H.row(1) = Hy.transpose();
	const SquareMatrix<float, 2> S = H * _P0 * H.transpose() + R * eye<float, 2>();
	const float rho = S(0, 1) / sqrtf(S(0, 0) * S(1, 1));
	ASSERT_GT(fabsf(rho), 0.3f) << "the prior does not couple the two flow components";
	const Vector2f innov(2.5f * sqrtf(innov_var(0)), (rho < 0.f ? 2.5f : -2.5f) * sqrtf(innov_var(1)));

	StateSample x_batch;
	SquareMatrixState P_batch;
	batchUpdate<2>(H, Vector2f(R, R), innov, x_batch, P_batch);

	ASSERT_TRUE(sequentialFlowUpdate(Hx, innov_var, R, innov));
	EXPECT_FALSE(flowAidSrc().innovation_rejected);

	// both components changed the state
	EXPECT_GT(fabsf(state().vel(0) - _x0.vel(0)), 0.05f);
	EXPECT_GT(fabsf(state().vel(1) - _x0.vel(1)), 0.05f);

	// the second component was not edited out after the first moved the state
	EXPECT_LT(flowAidSrc().test_ratio[0], 1.f);
	EXPECT_LT(flowAidSrc().test_ratio[1], 1.f);

	// sequential == batch
	const Vector25 dx = state().vector() - x_batch.vector();
	EXPECT_LT(dx.norm(), 1e-4f) << "state differs from the batch update by " << dx.norm();
	EXPECT_LT(maxAbsDiff(covariance(), P_batch), 1e-5f) << "covariance differs from the batch update by "
			<< maxAbsDiff(covariance(), P_batch);
}

TEST_F(EkfFlowFusionOrderTest, flowFusionEqualsBatchUpdateUncorrelatedVelocity)
{
	setPrior(Vector3f(0.8f, 1.2f, 0.f), 3.f, 0.f);
	const float R = 0.1f * 0.1f;

	VectorState Hx, Hy;
	Vector2f innov_var;
	lineariseFlow(R, Hx, Hy, innov_var);
	Matrix<float, 2, State::size> H;
	H.row(0) = Hx.transpose();
	H.row(1) = Hy.transpose();
	const Vector2f innov(1.5f * sqrtf(innov_var(0)), -2.f * sqrtf(innov_var(1)));

	StateSample x_batch;
	SquareMatrixState P_batch;
	batchUpdate<2>(H, Vector2f(R, R), innov, x_batch, P_batch);

	ASSERT_TRUE(sequentialFlowUpdate(Hx, innov_var, R, innov));
	EXPECT_LT(Vector25(state().vector() - x_batch.vector()).norm(), 1e-4f);
	EXPECT_LT(maxAbsDiff(covariance(), P_batch), 1e-5f);
}

TEST_F(EkfFlowFusionOrderTest, helperSequentialUpdateIsOrderInvariant)
{
	setPrior(Vector3f(1.f, 0.5f, -0.2f), 5.f, 0.5f);

	// two dense observation rows that share states
	VectorState h[2];
	h[0](State::vel.idx) = 0.7f;
	h[0](State::quat_nominal.idx + 2) = 0.2f;
	h[0](State::terrain.idx) = 0.5f;
	h[1](State::vel.idx) = -0.3f;
	h[1](State::vel.idx + 1) = 0.6f;
	h[1](State::quat_nominal.idx) = 0.4f;
	h[1](State::terrain.idx) = -0.2f;
	const float R[2] = {0.01f, 0.02f};
	const float innov[2] = {0.3f, -0.2f};

	Matrix<float, 2, State::size> H;
	H.row(0) = h[0].transpose();
	H.row(1) = h[1].transpose();
	StateSample x_batch;
	SquareMatrixState P_batch;
	batchUpdate<2>(H, Vector2f(R[0], R[1]), Vector2f(innov[0], innov[1]), x_batch, P_batch);

	const int order_a[2] = {0, 1};
	sequentialHelperUpdate(h, R, innov, order_a);
	const StateSample x_a = state();
	const SquareMatrixState P_a = covariance();

	const int order_b[2] = {1, 0};
	sequentialHelperUpdate(h, R, innov, order_b);
	const StateSample x_b = state();
	const SquareMatrixState P_b = covariance();

	EXPECT_LT(Vector25(x_a.vector() - x_b.vector()).norm(), 1e-5f) << "processing order changes the state";
	EXPECT_LT(maxAbsDiff(P_a, P_b), 1e-6f) << "processing order changes the covariance";
	EXPECT_LT(Vector25(x_a.vector() - x_batch.vector()).norm(), 1e-4f);
	EXPECT_LT(maxAbsDiff(P_a, P_batch), 1e-5f);
}
