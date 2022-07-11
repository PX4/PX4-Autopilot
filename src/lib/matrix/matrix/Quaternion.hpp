/**
 * @file Quaternion.hpp
 *
 * All rotations and axis systems follow the right-hand rule.
 * The Hamilton quaternion convention including its product definition is used.
 *
 * In order to rotate a vector in frame b (v_b) to frame n by a righthand
 * rotation defined by the quaternion q_nb (from frame b to n)
 * one can use the following operation:
 * v_n = q_nb * [0;v_b] * q_nb^(-1)
 *
 * Just like DCM's: v_n = C_nb * v_b (vector rotation)
 * M_n = C_nb * M_b * C_nb^(-1) (matrix rotation)
 *
 * or similarly the reverse operation
 * v_b = q_nb^(-1) * [0;v_n] * q_nb
 *
 * where q_nb^(-1) represents the inverse of the quaternion q_nb^(-1) = q_bn
 *
 * The product z of two quaternions z = q2 * q1 represents an intrinsic rotation
 * in the order of first q1 followed by q2.
 * The first element of the quaternion
 * represents the real part, thus, a quaternion representing a zero-rotation
 * is defined as (1,0,0,0).
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Euler;

template <typename Type>
class AxisAngle;


/**
 * Quaternion class
 *
 * The rotation between two coordinate frames is
 * described by this class.
 */
template<typename Type>
class Quaternion : public Vector<Type, 4>
{
public:
	using Matrix41 = Matrix<Type, 4, 1>;
	using Matrix31 = Matrix<Type, 3, 1>;

	/**
	 * Constructor from array
	 *
	 * @param data_ array
	 */
	explicit Quaternion(const Type data_[4]) :
		Vector<Type, 4>(data_)
	{
	}

	/**
	 * Standard constructor
	 */
	Quaternion()
	{
		Quaternion &q = *this;
		q(0) = 1;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
	}

	/**
	 * Constructor from Matrix41
	 *
	 * @param other Matrix41 to copy
	 */
	Quaternion(const Matrix41 &other) :
		Vector<Type, 4>(other)
	{
	}

	/**
	 * Constructor from dcm
	 *
	 * Instance is initialized from a dcm representing coordinate transformation
	 * from frame 2 to frame 1.
	 *
	 * @param dcm dcm to set quaternion to
	 */
	Quaternion(const Dcm<Type> &R)
	{
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

	/**
	 * Constructor from euler angles
	 *
	 * This sets the instance to a quaternion representing coordinate transformation from
	 * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
	 * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
	 *
	 * @param euler euler angle instance
	 */
	Quaternion(const Euler<Type> &euler)
	{
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

	/**
	 * Quaternion from AxisAngle
	 *
	 * @param aa axis-angle vector
	 */
	Quaternion(const AxisAngle<Type> &aa)
	{
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

	/**
	 * Quaternion from two vectors
	 * Generates shortest rotation from source to destination vector
	 *
	 * @param dst destination vector (no need to normalize)
	 * @param src source vector (no need to normalize)
	 * @param eps epsilon threshold which decides if a value is considered zero
	 */
	Quaternion(const Vector3<Type> &src, const Vector3<Type> &dst, const Type eps = Type(1e-5))
	{
		Quaternion &q = *this;
		Vector3<Type> cr = src.cross(dst);
		const float dt = src.dot(dst);

		if (cr.norm() < eps && dt < 0) {
			// handle corner cases with 180 degree rotations
			// if the two vectors are parallel, cross product is zero
			// if they point opposite, the dot product is negative
			cr = src.abs();

			if (cr(0) < cr(1)) {
				if (cr(0) < cr(2)) {
					cr = Vector3<Type>(1, 0, 0);

				} else {
					cr = Vector3<Type>(0, 0, 1);
				}

			} else {
				if (cr(1) < cr(2)) {
					cr = Vector3<Type>(0, 1, 0);

				} else {
					cr = Vector3<Type>(0, 0, 1);
				}
			}

			q(0) = Type(0);
			cr = src.cross(cr);

		} else {
			// normal case, do half-way quaternion solution
			q(0) = dt + std::sqrt(src.norm_squared() * dst.norm_squared());
		}

		q(1) = cr(0);
		q(2) = cr(1);
		q(3) = cr(2);
		q.normalize();
	}

	/**
	 * Constructor from quaternion values
	 *
	 * Instance is initialized from quaternion values representing coordinate
	 * transformation from frame 2 to frame 1.
	 * A zero-rotation quaternion is represented by (1,0,0,0).
	 *
	 * @param a set quaternion value 0
	 * @param b set quaternion value 1
	 * @param c set quaternion value 2
	 * @param d set quaternion value 3
	 */
	Quaternion(Type a, Type b, Type c, Type d)
	{
		Quaternion &q = *this;
		q(0) = a;
		q(1) = b;
		q(2) = c;
		q(3) = d;
	}

	/**
	 * Quaternion multiplication operator
	 *
	 * @param q quaternion to multiply with
	 * @return product
	 */
	Quaternion operator*(const Quaternion &p) const
	{
		const Quaternion &q = *this;
		return {
			q(0) *p(0) - q(1) *p(1) - q(2) *p(2) - q(3) *p(3),
			q(1) *p(0) + q(0) *p(1) - q(3) *p(2) + q(2) *p(3),
			q(2) *p(0) + q(3) *p(1) + q(0) *p(2) - q(1) *p(3),
			q(3) *p(0) - q(2) *p(1) + q(1) *p(2) + q(0) *p(3) };
	}

	/**
	 * Self-multiplication operator
	 *
	 * @param other quaternion to multiply with
	 */
	void operator*=(const Quaternion &other)
	{
		Quaternion &self = *this;
		self = self * other;
	}

	/**
	 * Scalar multiplication operator
	 *
	 * @param scalar scalar to multiply with
	 * @return product
	 */
	Quaternion operator*(Type scalar) const
	{
		const Quaternion &q = *this;
		return scalar * q;
	}

	/**
	 * Scalar self-multiplication operator
	 *
	 * @param scalar scalar to multiply with
	 */
	void operator*=(Type scalar)
	{
		Quaternion &q = *this;
		q = q * scalar;
	}

	/**
	 * Computes the derivative of q_21 when
	 * rotated with angular velocity expressed in frame 1
	 * v_2 = q_21 * v_1 * q_21^-1
	 * d/dt q_21 = 0.5 * q_21 * omega_2
	 *
	 * @param w angular rate in frame 1 (typically body frame)
	 */
	Matrix41 derivative1(const Matrix31 &w) const
	{
		const Quaternion &q = *this;
		Quaternion<Type> v(0, w(0, 0), w(1, 0), w(2, 0));
		return q * v  * Type(0.5);
	}

	/**
	 * Computes the derivative of q_21 when
	 * rotated with angular velocity expressed in frame 2
	 * v_2 = q_21 * v_1 * q_21^-1
	 * d/dt q_21 = 0.5 * omega_1 * q_21
	 *
	 * @param w angular rate in frame 2 (typically reference frame)
	 */
	Matrix41 derivative2(const Matrix31 &w) const
	{
		const Quaternion &q = *this;
		Quaternion<Type> v(0, w(0, 0), w(1, 0), w(2, 0));
		return v * q  * Type(0.5);
	}

	/**
	  * Computes the quaternion exponential of the 3D vector u
	  * as proposed in
	  * [1] Sveier A, Sjøberg AM, Egeland O. "Applied Runge–Kutta–Munthe-Kaas Integration
	  *     for the Quaternion Kinematics".Journal of Guidance, Control, and Dynamics. 2019
	  *
	  * return a quaternion computed as
	  * expq(u)=[cos||u||, sinc||u||*u]
	  * sinc(x)=sin(x)/x in the sin cardinal function
	  *
	  * This can be used to update a quaternion from the body rates
	  * rather than using
	  * qk+1=qk+qk.derivative1(wb)*dt
	  * we can use
	  * qk+1=qk*expq(dt*wb/2)
	  * which is a more robust update.
	  * A re-normalization step might necessary with both methods.
	  *
	  * @param u 3D vector u
	  */
	static Quaternion expq(const Vector3<Type> &u)
	{
		const Type tol = Type(0.2);           // ensures an error < 10^-10
		const Type c2 = Type(1.0 / 2.0);      // 1 / 2!
		const Type c3 = Type(1.0 / 6.0);      // 1 / 3!
		const Type c4 = Type(1.0 / 24.0);     // 1 / 4!
		const Type c5 = Type(1.0 / 120.0);    // 1 / 5!
		const Type c6 = Type(1.0 / 720.0);    // 1 / 6!
		const Type c7 = Type(1.0 / 5040.0);   // 1 / 7!

		Type u_norm = u.norm();
		Type sinc_u, cos_u;

		if (u_norm < tol) {
			Type u2 = u_norm * u_norm;
			Type u4 = u2 * u2;
			Type u6 = u4 * u2;

			// compute the first 4 terms of the Taylor serie
			sinc_u = Type(1.0) - u2 * c3 + u4 * c5 - u6 * c7;
			cos_u = Type(1.0) - u2 * c2 + u4 * c4 - u6 * c6;

		} else {
			sinc_u = Type(std::sin(u_norm) / u_norm);
			cos_u = Type(std::cos(u_norm));
		}

		Vector<Type, 3> v = sinc_u * u;
		return Quaternion<Type> (cos_u, v(0), v(1), v(2));
	}

	/** inverse right Jacobian of the quaternion logarithm u
	  * equation (20) in reference
	  * [1] Sveier A, Sjøberg AM, Egeland O. "Applied Runge–Kutta–Munthe-Kaas Integration
	  *     for the Quaternion Kinematics".Journal of Guidance, Control, and Dynamics. 2019
	  *
	  * This can be used to update a quaternion kinematic cleanly
	  * with higher order integration methods (like RK4) on the quaternion logarithm u.
	  *
	  * @param u 3D vector u
	  */
	static Dcm<Type> inv_r_jacobian(const Vector3<Type> &u)
	{
		const Type tol = Type(1.0e-4);
		Type u_norm = u.norm();
		Dcm<Type> u_hat = u.hat();

		if (u_norm < tol) { 	// result smaller than O(||.||^3)
			return Type(0.5) * (Dcm<Type>() + u_hat + (Type(1.0 / 3.0) + u_norm * u_norm / Type(45.0)) * u_hat * u_hat);

		} else {
			return Type(0.5) * (Dcm<Type>() + u_hat + (Type(1.0) - u_norm * Type(std::cos(u_norm) / std::sin(u_norm))) /
					    (u_norm * u_norm) * u_hat * u_hat);
		}
	}

	/**
	 * Invert quaternion in place
	 */
	void invert()
	{
		*this = this->inversed();
	}

	/**
	 * Invert quaternion
	 *
	 * @return inverted quaternion
	 */
	Quaternion inversed() const
	{
		const Quaternion &q = *this;
		Type normSq = q.dot(q);
		return Quaternion(
			       q(0) / normSq,
			       -q(1) / normSq,
			       -q(2) / normSq,
			       -q(3) / normSq);
	}

	/**
	 * Bring quaternion to canonical form
	 */
	void canonicalize()
	{
		*this = this->canonical();
	}

	/**
	 * Return canonical form of the quaternion
	 *
	 * @return quaternion in canonical from
	 */
	Quaternion canonical() const
	{
		const Quaternion &q = *this;

		for (size_t i = 0; i < 4; i++) {
			if (std::fabs(q(i)) > FLT_EPSILON) {
				return q * Type(matrix::sign(q(i)));
			}
		}

		return q;
	}

	/**
	 * Rotate quaternion from rotation vector
	 *
	 * @param vec rotation vector
	 */
	void rotate(const AxisAngle<Type> &vec)
	{
		Quaternion res(vec);
		(*this) = res * (*this);
	}

	/**
	 * Rotates vector v_1 in frame 1 to vector v_2 in frame 2
	 * using the rotation quaternion q_21
	 * describing the rotation from frame 1 to 2
	 * v_2 = q_21 * v_1 * q_21^-1
	 *
	 * @param vec vector to rotate in frame 1 (typically body frame)
	 * @return rotated vector in frame 2 (typically reference frame)
	 */
	Vector3<Type> rotateVector(const Vector3<Type> &vec) const
	{
		const Quaternion &q = *this;
		Quaternion v(Type(0), vec(0), vec(1), vec(2));
		Quaternion res = q * v * q.inversed();
		return Vector3<Type>(res(1), res(2), res(3));
	}

	/**
	 * Rotates vector v_2 in frame 2 to vector v_1 in frame 1
	 * using the rotation quaternion q_21
	 * describing the rotation from frame 1 to 2
	 * v_1 = q_21^-1 * v_2 * q_21
	 *
	 * @param vec vector to rotate in frame 2 (typically reference frame)
	 * @return rotated vector in frame 1 (typically body frame)
	 */
	Vector3<Type> rotateVectorInverse(const Vector3<Type> &vec) const
	{
		const Quaternion &q = *this;
		Quaternion v(Type(0), vec(0), vec(1), vec(2));
		Quaternion res = q.inversed() * v * q;
		return Vector3<Type>(res(1), res(2), res(3));
	}

	/**
	 * Imaginary components of quaternion
	 */
	Vector3<Type> imag() const
	{
		const Quaternion &q = *this;
		return Vector3<Type>(q(1), q(2), q(3));
	}

	/**
	 * Corresponding body z-axis to an attitude quaternion /
	 * last orthogonal unit basis vector
	 *
	 * == last column of the equivalent rotation matrix
	 * but calculated more efficiently than a full conversion
	 */
	Vector3<Type> dcm_z() const
	{
		const Quaternion &q = *this;
		Vector3<Type> R_z;
		const Type a = q(0);
		const Type b = q(1);
		const Type c = q(2);
		const Type d = q(3);
		R_z(0) = 2 * (a * c + b * d);
		R_z(1) = 2 * (c * d - a * b);
		R_z(2) = a * a - b * b - c * c + d * d;
		return R_z;
	}
};

using Quatf = Quaternion<float>;
using Quaternionf = Quaternion<float>;

using Quatd = Quaternion<double>;
using Quaterniond = Quaternion<double>;

} // namespace matrix
