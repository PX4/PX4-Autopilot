/**
 * @file MatrixSO3.hpp
 *
 * Inherited from matrix, written for geometric control and estimation scheme on SO(3).
 *
 * @author Ningshan Wang <nwang16@syr.edu>
 */


#pragma once

#include "math.hpp"

namespace matrix
{
	template <typename Type, size_t P, size_t Q, size_t M, size_t N>
	class Slice;

	template <typename Type, size_t M, size_t N>
	class Matrix;

	template<typename Type, size_t  M>
	class SquareMatrix;

	template <typename Type, size_t M>
	class Vector;

	template <typename Type>
	class Dcm;

	template <typename Type>
	class Vector2;

	template <typename Type>
	class Vector3;




	//typedef Vector3SO3<float> Vector3fSO3;

	template<typename Type>
	class MatrixSO3: public SquareMatrix<Type, 3> {
	public:
		MatrixSO3() = default;
		explicit MatrixSO3(const float data_[3][3]) :
			SquareMatrix<float, 3>(data_)
		{
		}

		explicit MatrixSO3(const float data_[3 * 3]) :
			SquareMatrix<float, 3>(data_)
		{
		}

		MatrixSO3(const SquareMatrix<float, 3> & other) :
			SquareMatrix<float, 3>(other)
		{
		}

		template<size_t P, size_t Q>
		MatrixSO3(const Slice<float, 3, 3, P, Q>& in_slice) : SquareMatrix<float, 3>(in_slice)
		{
		}

		MatrixSO3& operator=(const SquareMatrix<Type, 3> & other)
		{
			SquareMatrix<Type, 3>::operator=(other);
			return *this;
		}


		void quaternion2attitude(Vector<Type, 4> q) {
			MatrixSO3& self = *this;
			self(0, 0) = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
			self(0, 1) = 2 * (q(1) * q(2) - q(3) * q(0));
			self(0, 2) = 2 * (q(1) * q(3) + q(2) * q(0));
			self(1, 0) = 2 * (q(1) * q(2) + q(3) * q(0));
			self(1, 1) = 1 - 2 * (q(1) * q(1) + q(3) * q(3));
			self(1, 2) = 2 * (q(2) * q(3) - q(1) * q(0));
			self(2, 0) = 2 * (q(1) * q(3) - q(2) * q(0));
			self(2, 1) = 2 * (q(2) * q(3) + q(1) * q(0));
			self(2, 2) = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
			return;
		}



		Vector3<Type> sKgenerator() const {    // inverse to Dcm.vee() operation
			const MatrixSO3& Q(*this);
			Vector3<Type> sK;
			sK = 	Vector3<Type>(Q.transpose() * Vector3<Type>(1, 0, 0))%Vector3<Type>(1, 0, 0) * 1.2f
			+ 	Vector3<Type>(Q.transpose() * Vector3<Type>(0, 1, 0))%Vector3<Type>(0, 1, 0) * 1.1f
			+ 	Vector3<Type>(Q.transpose() * Vector3<Type>(0, 0, 1))%Vector3<Type>(0, 0, 1) * 1.0f;

			return sK;
		}


		Vector3<Type> wGenerator(Vector3<Type> omega) const {
			const MatrixSO3& Q(*this);
			Vector3<Type> wout;

			wout = 	Vector3<Type>(1, 0, 0)%(omega%Vector3<Type>(Q.transpose() * Vector3<Type>(1, 0, 0))) * 1.2f
			+ 	Vector3<Type>(0, 1, 0)%(omega%Vector3<Type>(Q.transpose() * Vector3<Type>(0, 1, 0))) * 1.1f
			+ 	Vector3<Type>(0, 0, 1)%(omega%Vector3<Type>(Q.transpose() * Vector3<Type>(0, 0, 1))) * 1.0f;

			return wout;
		}

		Vector3<Type> unskew() const {
			const MatrixSO3& wcross(*this);
			Vector3<Type> w;
			w(0) = wcross(2, 1);
			w(1) = wcross(0, 2);
			w(2) = wcross(1, 0);
			return w;
		}


		MatrixSO3<Type> expmso3(Vector3<Type> x) const {
			const MatrixSO3& R(*this);
			MatrixSO3<Type> Rnext;
			MatrixSO3<Type> F;
			SquareMatrix<Type,3> I;
			I.setIdentity();
			float nr = x.norm();
			if (nr<0.0001f){
				F=I;
				Rnext=R*F;			//Vector3<float> media1 = Q.transpose() * e1;
			//Vector3<float> media2 = Q.transpose() * e2;
			//Vector3<float> media3 = Q.transpose() * e3;

			//media1 = omega.cross(media1);
			//media2 = omega.cross(media2);
			//media3 = omega.cross(media3);

			}else{
				MatrixSO3 Sr;
				Sr = x.skew();
				F = I + (sin(nr))*Sr/nr +2.0f*float(pow(sin(0.5f*nr),2)/pow(nr,2))*Sr*Sr;
				Rnext = R*F;
			}
			return Rnext;;
		}




/*
		Vector3<float> logmso3() const {
			const Matrix3fSO3& Q(*this);
			Matrix3fSO3 media;
			Matrix3fSO3 A;
			Matrix3fSO3 AA;
			Matrix3fSO3 S;
			Matrix3fSO3 SS;
			//media.setIdentity();
			Matrix3fSO3 logR;
			//logR.setIdentity();
			Vector3<float> r(0, 0, 0);
			float ri;
			float pi = 3.1416;
			float trR = Q(0, 0) + Q(1, 1) + Q(2, 2);
			float phi = acos(0.5f * (trR - 1));
			A = Q - Q.transpose();
			AA = A.transpose() * A;
			float nA = sqrt(AA(0, 0) + AA(1, 1) + AA(2, 2));
			S = 0.5f * (Q - I_);
			SS = S.transpose() * S;
			Vector3<float> sd(S(0, 0), S(1, 1), S(2, 2));
			float nS = sqrt(SS(0, 0) + SS(1, 1) + SS(2, 2));
			int max_i = 0;
			float max = sd(0);
			for (int i = 0; i <= 2; i++) {
				if (sd(i) > max) {
					max = sd(i);
					max_i = i;
				}
			}
			if (nA < 0.0005f && nS < 0.0005f) {
				r = Vector3<float>(	vehicle_angular_velocity_s angular_velocity;0.0f, 0.0f, 0.0f);
			}
			else if (nA < 0.0005f && nS > 0.05f) {
				ri = sqrt(1.0f + max);
				if (max_i == 0) {
					r = Vector3<float>(ri, S(0, 1) / ri, S(0, 2) / ri) * pi;
				}
				else if (max_i == 1) {
					r = Vector3<float>(S(1, 0) / ri, ri, S(1, 2) / ri) * pi;
				}
				else {
					r = Vector3<float>(S(2, 0) / ri, S(2, 1) / ri, ri) * pi;
				}
			}
			else {
				media = Q - Q.transpose();
				logR = media * media.cs(phi);
				r = logR.unskew();
			}
			return r;
		}
*/




	};
	using MatrixfSO3 = MatrixSO3<float>;
	using MatrixdSO3 = MatrixSO3<double>;
	//typedef Matrix3SO3<float> Matrix3fSO3;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
