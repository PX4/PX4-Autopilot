// Out-of-line definitions of the float rotation-representation conversion
// constructors, declared as explicit specializations in matrix/math.hpp.
// Bodies are verbatim copies of the primary templates (Type = float) from the
// respective headers; keeping them out of line saves flash by avoiding
// inlining into every call site.
#include "math.hpp"

namespace matrix
{

template<> Dcm<float>::Dcm(const Quaternion<float> &q)
{
	using Type = float;
	Dcm &dcm = *this;
	const Type a = q(0);
	const Type b = q(1);
	const Type c = q(2);
	const Type d = q(3);
	const Type ab = a * b;
	const Type ac = a * c;
	const Type ad = a * d;
	const Type bb = b * b;
	const Type bc = b * c;
	const Type bd = b * d;
	const Type cc = c * c;
	const Type cd = c * d;
	const Type dd = d * d;
	dcm(0, 0) = Type(1) - Type(2) * (cc + dd);
	dcm(0, 1) = Type(2) * (bc - ad);
	dcm(0, 2) = Type(2) * (ac + bd);
	dcm(1, 0) = Type(2) * (bc + ad);
	dcm(1, 1) = Type(1) - Type(2) * (bb + dd);
	dcm(1, 2) = Type(2) * (cd - ab);
	dcm(2, 0) = Type(2) * (bd - ac);
	dcm(2, 1) = Type(2) * (ab + cd);
	dcm(2, 2) = Type(1) - Type(2) * (bb + cc);
}

template<> Dcm<float>::Dcm(const Euler<float> &euler)
{
	using Type = float;
	Dcm &dcm = *this;
	Type cosPhi = Type(std::cos(euler.phi()));
	Type sinPhi = Type(std::sin(euler.phi()));
	Type cosThe = Type(std::cos(euler.theta()));
	Type sinThe = Type(std::sin(euler.theta()));
	Type cosPsi = Type(std::cos(euler.psi()));
	Type sinPsi = Type(std::sin(euler.psi()));

	dcm(0, 0) = cosThe * cosPsi;
	dcm(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
	dcm(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

	dcm(1, 0) = cosThe * sinPsi;
	dcm(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
	dcm(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

	dcm(2, 0) = -sinThe;
	dcm(2, 1) = sinPhi * cosThe;
	dcm(2, 2) = cosPhi * cosThe;
}

template<> Euler<float>::Euler(const Dcm<float> &dcm)
{
	using Type = float;
	theta() = std::asin(-dcm(2, 0));

	if ((std::fabs(theta() - Type(M_PI / 2))) < Type(1.0e-3)) {
		phi() = 0;
		psi() = std::atan2(dcm(1, 2), dcm(0, 2));

	} else if ((std::fabs(theta() + Type(M_PI / 2))) < Type(1.0e-3)) {
		phi() = 0;
		psi() = std::atan2(-dcm(1, 2), -dcm(0, 2));

	} else {
		phi() = std::atan2(dcm(2, 1), dcm(2, 2));
		psi() = std::atan2(dcm(1, 0), dcm(0, 0));
	}
}

template<> Euler<float>::Euler(const Quaternion<float> &q) : Vector<float, 3>(Euler(Dcm<float>(q)))
{
}

template<> Quaternion<float>::Quaternion(const Dcm<float> &R)
{
	using Type = float;
	Quaternion &q = *this;
	Type t = R.trace();

	if (t > Type(0)) {
		t = std::sqrt(Type(1) + t);
		q(0) = Type(0.5) * t;
		t = Type(0.5) / t;
		q(1) = (R(2, 1) - R(1, 2)) * t;
		q(2) = (R(0, 2) - R(2, 0)) * t;
		q(3) = (R(1, 0) - R(0, 1)) * t;

	} else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
		t = std::sqrt(Type(1) + R(0, 0) - R(1, 1) - R(2, 2));
		q(1) = Type(0.5) * t;
		t = Type(0.5) / t;
		q(0) = (R(2, 1) - R(1, 2)) * t;
		q(2) = (R(1, 0) + R(0, 1)) * t;
		q(3) = (R(0, 2) + R(2, 0)) * t;

	} else if (R(1, 1) > R(2, 2)) {
		t = std::sqrt(Type(1) - R(0, 0) + R(1, 1) - R(2, 2));
		q(2) = Type(0.5) * t;
		t = Type(0.5) / t;
		q(0) = (R(0, 2) - R(2, 0)) * t;
		q(1) = (R(1, 0) + R(0, 1)) * t;
		q(3) = (R(2, 1) + R(1, 2)) * t;

	} else {
		t = std::sqrt(Type(1) - R(0, 0) - R(1, 1) + R(2, 2));
		q(3) = Type(0.5) * t;
		t = Type(0.5) / t;
		q(0) = (R(1, 0) - R(0, 1)) * t;
		q(1) = (R(0, 2) + R(2, 0)) * t;
		q(2) = (R(2, 1) + R(1, 2)) * t;
	}
}

template<> Quaternion<float>::Quaternion(const Euler<float> &euler)
{
	using Type = float;
	Quaternion &q = *this;
	Type cosPhi_2 = Type(std::cos(euler.phi() / Type(2)));
	Type cosTheta_2 = Type(std::cos(euler.theta() / Type(2)));
	Type cosPsi_2 = Type(std::cos(euler.psi() / Type(2)));
	Type sinPhi_2 = Type(std::sin(euler.phi() / Type(2)));
	Type sinTheta_2 = Type(std::sin(euler.theta() / Type(2)));
	Type sinPsi_2 = Type(std::sin(euler.psi() / Type(2)));
	q(0) = cosPhi_2 * cosTheta_2 * cosPsi_2 +
	       sinPhi_2 * sinTheta_2 * sinPsi_2;
	q(1) = sinPhi_2 * cosTheta_2 * cosPsi_2 -
	       cosPhi_2 * sinTheta_2 * sinPsi_2;
	q(2) = cosPhi_2 * sinTheta_2 * cosPsi_2 +
	       sinPhi_2 * cosTheta_2 * sinPsi_2;
	q(3) = cosPhi_2 * cosTheta_2 * sinPsi_2 -
	       sinPhi_2 * sinTheta_2 * cosPsi_2;
}

template<> Quaternion<float>::Quaternion(const AxisAngle<float> &aa)
{
	using Type = float;
	Quaternion &q = *this;
	Type angle = aa.norm();
	Vector<Type, 3> axis = aa.unit();

	if (angle < Type(1e-10)) {
		q(0) = Type(1);
		q(1) = q(2) = q(3) = 0;

	} else {
		Type magnitude = std::sin(angle / Type(2));
		q(0) = std::cos(angle / Type(2));
		q(1) = axis(0) * magnitude;
		q(2) = axis(1) * magnitude;
		q(3) = axis(2) * magnitude;
	}
}

template<> AxisAngle<float>::AxisAngle(const Quaternion<float> &q)
{
	using Type = float;
	AxisAngle &v = *this;
	Type mag = q.imag().norm();

	if (std::fabs(mag) >= Type(1e-10)) {
		v = q.imag() * Type(Type(2) * std::atan2(mag, q(0)) / mag);

	} else {
		v = q.imag() * Type(Type(2) * Type(sign(q(0))));
	}
}

} // namespace matrix
