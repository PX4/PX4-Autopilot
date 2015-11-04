// dummy code to instantiate all templates for coverage

#include <Matrix.hpp>
#include <Vector.hpp>
#include <Vector3.hpp>
#include <Euler.hpp>
#include <Scalar.hpp>
#include <Quaternion.hpp>

namespace matrix {

template class Vector<float, 2>;
template class Vector<float, 3>;
template class Vector<float, 4>;
template class Euler<float>;
template class Scalar<float>;
template class Matrix<float, 3, 3>;
template class Matrix<float, 1, 1>;
template class Quaternion<float>;

};
