// dummy code to instantiate all templates for coverage

#include <Matrix.hpp>
#include <Vector.hpp>
#include <Vector3.hpp>
#include <Euler.hpp>
#include <Scalar.hpp>
#include <Quaternion.hpp>

template Vector<float, 2>;
template Euler<float>;
template Scalar<float>;
template Matrix<float, 3, 3>;
