# matrix [![Build Status](https://travis-ci.org/PX4/Matrix.svg?branch=master)](https://travis-ci.org/PX4/Matrix) [![Coverage Status](https://coveralls.io/repos/PX4/Matrix/badge.svg?branch=master&service=github)](https://coveralls.io/github/PX4/Matrix?branch=master)

A simple and efficient template based matrix library.

## License
* (BSD) The Matrix library is licensed under a permissive 3-clause BSD license. Contributions must be made under the same license.

## Features
* Compile time size checking.
* Euler angle (321), DCM, Quaternion conversion through constructors.
* High testing coverage. 

## Limitations
* No dynamically sized matrices.

## Library Overview

* matrix/math.hpp : Provides matrix math routines.
	* Matrix (MxN)
	* Square Matrix (MxM, has inverse)
	* Vector (Mx1)
	* Scalar (1x1)
	* Quaternion
	* Dcm
	* Euler (Body 321)

* matrix/filter.hpp : Provides filtering routines.
	* kalman_correct

* matrix/integrate.hpp : Provides integration routines.
	* integrate_rk4 (Runge-Kutta 4th order)

## Example

See the test directory for detailed examples. Some simple examples are included below:

```c++
	// define an euler angle (Body 3(yaw)-2(pitch)-1(roll) rotation)
	float roll = 0.1f;
	float pitch = 0.2f;
	float yaw = 0.3f;
	Eulerf euler(roll, pitch, yaw);

	// convert to quaternion from euler
	Quatf q_nb(euler);

	// convert to DCM from quaternion
	Dcmf dcm(q_nb);

	// do some kalman filtering
	const size_t n_x = 5;
	const size_t n_y = 3;

	// define matrix sizes
	SquareMatrix<float, n_x> P;
	Vector<float, n_x> x;
	Vector<float, n_y> y;
	Matrix<float, n_y, n_x> C;
	SquareMatrix<float, n_y> R;
	SquareMatrix<float, n_y> S;
	Matrix<float, n_x, n_y> K;

	// define measurement matrix
	C = zero<float, n_y, n_x>(); // or C.setZero()
	C(0,0) = 1;
	C(1,1) = 2;
	C(2,2) = 3;

	// set x to zero
	x = zero<float, n_x, 1>(); // or x.setZero()

	// set P to identity * 0.01
	P = eye<float, n_x>()*0.01;

	// set R to identity * 0.1
	R = eye<float, n_y>()*0.1;

	// measurement
	y(0) = 1;
	y(1) = 2;
	y(2) = 3;

	// innovation
	r = y - C*x;

	// innovations variance
	S = C*P*C.T() + R;

	// Kalman gain matrix
	K = P*C.T()*S.I();
	// S.I() is the inverse, defined for SquareMatrix

	// correction
	x += K*r;

	// slicing
    float data[9] = {0, 2, 3,
                     4, 5, 6,
                     7, 8, 10
                    };
    SquareMatrix<float, 3> A(data);

	// Slice a 3,3 matrix starting at row 1, col 0,
	// with size 2 x 3, warning, no size checking
    Matrix<float, 2, 3> B(A.slice<2, 3>(1, 0));

	// this results in:
	// 4, 5, 6
	// 7, 8, 10
```
