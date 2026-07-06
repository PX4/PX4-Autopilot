#pragma once

#include "AxisAngle.hpp"
#include "Dcm.hpp"
#include "Dcm2.hpp"
#include "Dual.hpp"
#include "Euler.hpp"
#include "helper_functions.hpp"
#include "LeastSquaresSolver.hpp"
#include "Matrix.hpp"
#include "PseudoInverse.hpp"
#include "Quaternion.hpp"
#include "Scalar.hpp"
#include "Slice.hpp"
#include "SparseVector.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"

namespace matrix
{
// Explicit specializations of the float rotation-representation conversions.
// Defined out-of-line (in mathlib) to save flash: the conversion bodies are
// large and would otherwise be inlined into every call site.
template<> Dcm<float>::Dcm(const Quaternion<float> &q);
template<> Dcm<float>::Dcm(const Euler<float> &euler);
template<> Euler<float>::Euler(const Dcm<float> &dcm);
template<> Euler<float>::Euler(const Quaternion<float> &q);
template<> Quaternion<float>::Quaternion(const Dcm<float> &R);
template<> Quaternion<float>::Quaternion(const Euler<float> &euler);
template<> Quaternion<float>::Quaternion(const AxisAngle<float> &aa);
template<> AxisAngle<float>::AxisAngle(const Quaternion<float> &q);
} // namespace matrix
