#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2023-2026 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: trig_series_derivation.py
Description:
    Generates generated/trig_series.h, the small-angle-safe evaluation of the
    three coefficients of the closed-form strapdown INS propagation.

    Each coefficient is a ratio whose numerator is a difference of nearly equal
    terms, so evaluating it directly loses all significance to cancellation at
    the rotation angles a 250 Hz IMU actually produces. The generated code
    switches to the Maclaurin series below a threshold derived here.
"""

import numpy as np
import sympy as sp

FLOAT32_EPS = float(np.finfo(np.float32).eps)

# Worst-case relative error we accept over the whole range of theta, in units of
# FLOAT32_EPS. The cancellation-prone closed forms only recover slowly as theta
# grows, so demanding much less than this pushes the switch past the point where
# a short series is still accurate.
TARGET_REL_ERR = 16 * FLOAT32_EPS

MAX_SERIES_TERMS = 8

theta = sp.symbols("theta", positive=True)

# name, exact expression, factorial offset k of the series sum (-1)^n theta^2n / (2n+k)!,
# the definition as text, and the same expression as C++ in terms of theta and theta_sq
COEFFICIENTS = [
    ("c1", (1 - sp.cos(theta)) / theta**2, 2,
     "(1 - cos(theta)) / theta^2",
     "(T(1) - std::cos(theta)) / theta_sq"),
    ("c2", (theta - sp.sin(theta)) / theta**3, 3,
     "(theta - sin(theta)) / theta^3",
     "(theta - std::sin(theta)) / (theta_sq * theta)"),
    ("c3", (theta**2 / 2 + sp.cos(theta) - 1) / theta**4, 4,
     "(theta^2 / 2 + cos(theta) - 1) / theta^4",
     "(T(0.5) * theta_sq + std::cos(theta) - T(1)) / (theta_sq * theta_sq)"),
]


def series_coefficients(expr, k, n_terms):
    """Maclaurin coefficients in theta^2, cross-checked against (-1)^n / (2n+k)!"""
    expansion = sp.series(expr, theta, 0, 2 * n_terms).removeO()
    coefficients = []

    for n in range(n_terms):
        coefficient = expansion.coeff(theta, 2 * n)
        assert sp.simplify(coefficient - sp.Rational((-1)**n, sp.factorial(2 * n + k))) == 0, \
            "series of {} disagrees with (-1)^n/(2n+{})! at n={}".format(expr, k, n)
        coefficients.append(coefficient)

    return coefficients


def reference_values(thetas, expr):
    import mpmath
    mpmath.mp.dps = 40
    f = sp.lambdify(theta, expr, "mpmath")
    return np.array([float(f(mpmath.mpf(float(t)))) for t in thetas])


def series_values_float32(coefficients, thetas_sq):
    """Horner in theta^2, exactly as the generated C++ evaluates it"""
    acc = np.full(thetas_sq.shape, np.float32(coefficients[-1]), dtype=np.float32)

    for coefficient in reversed(coefficients[:-1]):
        acc = np.float32(coefficient) + thetas_sq * acc

    return acc


def closed_form_values_float32(expr, thetas):
    """The cancellation happens in the arithmetic, so the whole evaluation must be single precision"""
    f = sp.lambdify(theta, expr, "numpy")
    values = f(thetas.astype(np.float32))
    assert values.dtype == np.float32, "closed form was not evaluated in single precision"
    return values


def select_series(expr, k, thetas, references):
    """Shortest series that meets TARGET_REL_ERR, with the threshold that minimises the error"""
    thetas_sq = (thetas * thetas).astype(np.float32)
    closed_form_error = np.abs(closed_form_values_float32(expr, thetas) - references) / np.abs(references)
    candidate_thresholds = np.round(np.arange(0.05, 3.0, 0.01), 2)
    best = None

    for n_terms in range(2, MAX_SERIES_TERMS + 1):
        coefficients = series_coefficients(expr, k, n_terms)
        series_error = np.abs(series_values_float32(coefficients, thetas_sq) - references) / np.abs(references)

        for threshold in candidate_thresholds:
            error = np.where(thetas_sq < np.float32(threshold**2), series_error, closed_form_error)
            worst = float(error.max())

            if best is None or worst < best[0]:
                best = (worst, float(thetas[error.argmax()]), n_terms, threshold, coefficients)

        if best[0] <= TARGET_REL_ERR and best[2] == n_terms:
            break

    return best


def render_horner(coefficients):
    expression = "T({})".format(sp.printing.ccode(sp.nsimplify(coefficients[-1])))

    for coefficient in reversed(coefficients[:-1]):
        expression = "T({}) + theta_sq * ({})".format(sp.printing.ccode(sp.nsimplify(coefficient)), expression)

    return expression


HEADER = """// --------------------------------------------------
// This file was autogenerated, do NOT modify by hand
// --------------------------------------------------

#ifndef EKF_TRIG_SERIES_H
#define EKF_TRIG_SERIES_H

#include <cmath>

namespace math
{{
namespace trig_series
{{

// Coefficients of the closed-form strapdown INS propagation of
// Goppert, James, et al. "A Closed-form Solution for the Strapdown Inertial Navigation Initial Value Problem."
// arXiv preprint arXiv:2310.04886 (2023), where theta is the rotation angle over the integration interval [rad].
//
// Each is the sum (-1)^n theta^2n / (2n+k)! derived in the proof of Theorem 1, with k = 2, 3 and 4
// respectively. The numerator of every closed form is a difference of nearly equal terms, so below
// the switch thresholds the series is used instead; the thresholds and series lengths are chosen by
// trig_series_derivation.py to bound the relative error over all theta.

{bodies}
}} // namespace trig_series
}} // namespace math

#endif // !EKF_TRIG_SERIES_H
"""

BODY = """// {definition}
// Series below theta = {threshold}, worst-case relative error {worst:.2e} at theta = {worst_theta:.3f}
template <typename T>
T {name}(const T theta_sq)
{{
	if (theta_sq < T({threshold_sq})) {{
		return {series};
	}}

	const T theta = std::sqrt(theta_sq);
	return {closed_form};
}}
"""


def main():
    thetas = np.concatenate([np.logspace(-7, 0, 1500), np.linspace(1.0, 3.2, 500)[1:]])
    bodies = []

    for name, expr, k, definition, closed_form in COEFFICIENTS:
        references = reference_values(thetas, expr)
        worst, worst_theta, n_terms, threshold, coefficients = select_series(expr, k, thetas, references)

        print("  |- {}: {} terms, switch at theta = {:.2f}, worst relative error {:.2e} ({:.1f} eps)".format(
            name, n_terms, threshold, worst, worst / FLOAT32_EPS))

        bodies.append(BODY.format(
            name=name,
            definition=definition,
            worst=worst,
            worst_theta=worst_theta,
            threshold=threshold,
            threshold_sq="{:.6g}".format(threshold**2),
            series=render_horner(coefficients),
            closed_form=closed_form))

    with open("generated/trig_series.h", "w") as f:
        f.write(HEADER.format(bodies="\n".join(bodies)))

    print("  |- trig_series.h")


if __name__ == "__main__":
    print("Generate trig series")
    main()
