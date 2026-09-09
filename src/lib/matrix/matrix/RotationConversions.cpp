// Out-of-line definitions of the float rotation-representation conversion
// constructors, declared as explicit specializations in matrix/math.hpp.
// Keeping them out of line saves flash by avoiding inlining into every call
// site. The actual math lives once in the matrix::detail helper templates (in
// the respective headers); these specializations just delegate to it, so there
// is no code duplication with the primary (in-line) constructors.
#include "math.hpp"

namespace matrix
{

template<> Dcm<float>::Dcm(const Quaternion<float> &q)
{
	detail::dcm_from_quaternion(*this, q);
}

template<> Dcm<float>::Dcm(const Euler<float> &euler)
{
	detail::dcm_from_euler(*this, euler);
}

template<> Euler<float>::Euler(const Dcm<float> &dcm)
{
	detail::euler_from_dcm(*this, dcm);
}

template<> Euler<float>::Euler(const Quaternion<float> &q) : Vector<float, 3>(Euler(Dcm<float>(q)))
{
}

template<> Quaternion<float>::Quaternion(const Dcm<float> &R)
{
	detail::quaternion_from_dcm(*this, R);
}

template<> Quaternion<float>::Quaternion(const Euler<float> &euler)
{
	detail::quaternion_from_euler(*this, euler);
}

template<> Quaternion<float>::Quaternion(const AxisAngle<float> &aa)
{
	detail::quaternion_from_axis_angle(*this, aa);
}

template<> AxisAngle<float>::AxisAngle(const Quaternion<float> &q)
{
	detail::axis_angle_from_quaternion(*this, q);
}

} // namespace matrix
