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
 * Symbolic function: computeInnovCov
 *
 * Args:
 *     meas_unc: Scalar
 *     covariance: Matrix44
 *     meas_matrix: Matrix14
 *
 * Outputs:
 *     innov_cov_updated: Scalar
 */
template <typename Scalar>
void Computeinnovcov(const Scalar meas_unc, const matrix::Matrix<Scalar, 4, 4> &covariance,
		     const matrix::Matrix<Scalar, 1, 4> &meas_matrix,
		     Scalar *const innov_cov_updated = nullptr)
{
	// Total ops: 36

	// Input arrays

	// Intermediate terms (0)

	// Output terms (1)
	if (innov_cov_updated != nullptr) {
		Scalar &_innov_cov_updated = (*innov_cov_updated);

		_innov_cov_updated =
			meas_matrix(0, 0) *
			(covariance(0, 0) * meas_matrix(0, 0) + covariance(1, 0) * meas_matrix(0, 1) +
			 covariance(2, 0) * meas_matrix(0, 2) + covariance(3, 0) * meas_matrix(0, 3)) +
			meas_matrix(0, 1) *
			(covariance(0, 1) * meas_matrix(0, 0) + covariance(1, 1) * meas_matrix(0, 1) +
			 covariance(2, 1) * meas_matrix(0, 2) + covariance(3, 1) * meas_matrix(0, 3)) +
			meas_matrix(0, 2) *
			(covariance(0, 2) * meas_matrix(0, 0) + covariance(1, 2) * meas_matrix(0, 1) +
			 covariance(2, 2) * meas_matrix(0, 2) + covariance(3, 2) * meas_matrix(0, 3)) +
			meas_matrix(0, 3) *
			(covariance(0, 3) * meas_matrix(0, 0) + covariance(1, 3) * meas_matrix(0, 1) +
			 covariance(2, 3) * meas_matrix(0, 2) + covariance(3, 3) * meas_matrix(0, 3)) +
			meas_unc;
	}
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym