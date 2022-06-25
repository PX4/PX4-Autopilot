




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

	//template<typename Type>
	class Matrix3fSO3: public SquareMatrix<float, 3> {
	private:
		float IdentityArray[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		//float e1Array[3] = { 1.0f, 0.0f, 0.0f};
		//float e2Array[3] = { 0.0f, 1.0f, 0.0f};
		//float e3Array[3] = { 0.0f, 0.0f, 1.0f};
		//SquareMatrix<float, 3> Identity{ IdentityArray };
		SquareMatrix<float, 3> Identity=SquareMatrix<float, 3>(IdentityArray);

		Vector3<float> e1{1.0f, 0.0f, 0.0f};
		Vector3<float> e2{0.0f, 1.0f, 0.0f};
		Vector3<float> e3{0.0f, 0.0f, 1.0f};

		/*
		float cs(float x) {
			float result = 0;
			float input;
			input = (double)x;
			if (x <= 0.2f && x >= -0.2f) {
				result = 1 * 1.0f / 2.0f + (float)pow(x, 2) / 12.0f + (7 * (float)pow(x, 4)) / 720.0f + (31 * (float)pow(x, 6)) / 30240.0f;
			}
			else {
				result = input * (float)pow(sin(input), -1) * 1.0f / 2.0f;
			}

			return result;
		}
		*/
	public:
		Matrix3fSO3() = default;
		explicit Matrix3fSO3(const float data_[3][3]) :
			SquareMatrix<float, 3>(data_)
		{
		}

		explicit Matrix3fSO3(const float data_[3 * 3]) :
			SquareMatrix<float, 3>(data_)
		{
		}

		Matrix3fSO3(const SquareMatrix<float, 3> & other) :
			SquareMatrix<float, 3>(other)
		{
		}

		template<size_t P, size_t Q>
		Matrix3fSO3(const Slice<float, 3, 3, P, Q>& in_slice) : SquareMatrix<float, 3>(in_slice)
		{
		}

		Matrix3fSO3& operator=(const SquareMatrix<float, 3> & other)
		{
			SquareMatrix<float, 3>::operator=(other);
			return *this;
		}

		void diag(Vector3<float> d) {
			Matrix3fSO3& self = *this;
			for (int i = 0; i < 3; i++) {
				self(i, i) = d(i);
			}
			return;
		}

		void quaternion2attitude(Vector<float, 4> q) {
			Matrix3fSO3& self = *this;
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



		Vector3<float> sKgenerator() const {    // inverse to Dcm.vee() operation
			const Matrix3fSO3& Q(*this);
			Vector3<float> sK;
			sK = Vector3<float>(Q.transpose() * e1)%e1 * 1.2f + Vector3<float>(Q.transpose() * e2)%e2 * 1.1f + Vector3<float>(Q.transpose() * e3)%e3 * 1.0f;

			return sK;
		}


		Vector3<float> wGenerator(Vector3<float> omega) const {
			const Matrix3fSO3& Q(*this);
			Vector3<float> wout;

			//Vector3<float> media1 = Q.transpose() * e1;
			//Vector3<float> media2 = Q.transpose() * e2;
			//Vector3<float> media3 = Q.transpose() * e3;

			//media1 = omega.cross(media1);
			//media2 = omega.cross(media2);
			//media3 = omega.cross(media3);

			wout = e1%(omega%Vector3<float>(Q.transpose() * e1)) * 1.2f + e2%(omega%Vector3<float>(Q.transpose() * e2)) * 1.1f + e3%(omega%Vector3<float>(Q.transpose() * e3)) * 1.0f;

			return wout;
		}

		Vector3<float> zKGenerator(Vector3<float> x, float p_) const {
			const Matrix3fSO3& Y(*this);
			Matrix<float, 3, 1> x_;
			Matrix<float, 1, 1> domm;
			Vector3<float> output;
			float dom;
			x_.setCol(0, x);
			domm = x_.transpose() * Y * x_;
			dom = pow(domm(0, 0), 1 - pow(p_, -1));
			output = x / dom;
			return output;
		}


		Vector3<float> unskew() const {
			const Matrix3fSO3& wcross(*this);
			Vector3<float> w;
			w(0) = wcross(2, 1);
			w(1) = wcross(0, 2);
			w(2) = wcross(1, 0);
			return w;
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
				r = Vector3<float>(0.0f, 0.0f, 0.0f);
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
	//typedef Matrix3SO3<float> Matrix3fSO3;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
