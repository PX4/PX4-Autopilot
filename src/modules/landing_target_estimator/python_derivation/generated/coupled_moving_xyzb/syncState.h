// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym
{

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: syncState
 *
 * Args:
 *     dt: Scalar
 *     state: Matrix12_1
 *     acc: Matrix31
 *
 * Outputs:
 *     state_updated: Matrix12_1
 */
template <typename Scalar>
void Syncstate(const Scalar dt, const matrix::Matrix<Scalar, 12, 1> &state,
	       const matrix::Matrix<Scalar, 3, 1> &acc,
	       matrix::Matrix<Scalar, 12, 1> *const state_updated = nullptr)
{
	// Total ops: 45

	// Input arrays

	// Intermediate terms (8)
	const Scalar _tmp0 = Scalar(0.5) * std::pow(dt, Scalar(2));
	const Scalar _tmp1 = Scalar(1.0) * state(3, 0);
	const Scalar _tmp2 = Scalar(1.0) * state(4, 0);
	const Scalar _tmp3 = Scalar(1.0) * state(5, 0);
	const Scalar _tmp4 = Scalar(1.0) * dt;
	const Scalar _tmp5 = Scalar(1.0) * state(9, 0);
	const Scalar _tmp6 = Scalar(1.0) * state(10, 0);
	const Scalar _tmp7 = Scalar(1.0) * state(11, 0);

	// Output terms (1)
	if (state_updated != nullptr) {
		matrix::Matrix<Scalar, 12, 1> &_state_updated = (*state_updated);

		_state_updated(0, 0) =
			-_tmp0 * acc(0, 0) + _tmp0 * state(9, 0) - _tmp1 * dt + Scalar(1.0) * state(0, 0);
		_state_updated(1, 0) =
			-_tmp0 * acc(1, 0) + _tmp0 * state(10, 0) - _tmp2 * dt + Scalar(1.0) * state(1, 0);
		_state_updated(2, 0) =
			-_tmp0 * acc(2, 0) + _tmp0 * state(11, 0) - _tmp3 * dt + Scalar(1.0) * state(2, 0);
		_state_updated(3, 0) = _tmp1 + _tmp4 * acc(0, 0) - _tmp5 * dt;
		_state_updated(4, 0) = _tmp2 + _tmp4 * acc(1, 0) - _tmp6 * dt;
		_state_updated(5, 0) = _tmp3 + _tmp4 * acc(2, 0) - _tmp7 * dt;
		_state_updated(6, 0) = Scalar(1.0) * state(6, 0);
		_state_updated(7, 0) = Scalar(1.0) * state(7, 0);
		_state_updated(8, 0) = Scalar(1.0) * state(8, 0);
		_state_updated(9, 0) = _tmp5;
		_state_updated(10, 0) = _tmp6;
		_state_updated(11, 0) = _tmp7;
	}
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym