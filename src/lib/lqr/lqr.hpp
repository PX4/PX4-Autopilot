/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef LQR_H_
#define LQR_H_

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace matrix;

//TODO should be in Matrix utils?
// see https://github.com/PX4/PX4-Autopilot/blob/0607982b234f4ddc0f8037829fcb051d8cb34b98/src/lib/matrix/matrix/Matrix.hpp#L803-L821
// why it is under SUPPORT_STDIOSTREAM?
template<typename Type, size_t M, size_t N>
void read_matrix(std::string file_path, Matrix<Type, M, N> &matrix_out) {

	static std::ifstream infile;
	infile.open(file_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < M; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < N; column++) {
				this_stream >> matrix_out(row, column);
			}
		}
		infile.close();
	} else
		PX4_ERR("K.txt not opened");
}

template<size_t STATE_DIM, size_t CONTROL_DIM>
class Lqr
{
public:

	Lqr() {set_K("K.txt");}
	~Lqr() = default;

	bool update(const Vector<float, STATE_DIM> &x, const Vector<float, STATE_DIM> &x_sp, Vector<float, CONTROL_DIM> &u)
	{
		Vector<float, STATE_DIM> delta_x = x_sp - x;
		u = _K * delta_x;

		return true;
	}

	void set_K(std::string file_path) {
		read_matrix(file_path, _K);
	}

private:
	// Matrix<float, STATE_DIM, STATE_DIM> _A;
	// Matrix<float, STATE_DIM, CONTROL_DIM> _B;
	Matrix<float, CONTROL_DIM, STATE_DIM> _K;

	// Vector<float, STATE_DIM> _x;
	// Vector<float, CONTROL_DIM> _u;
	// Vector<float, STATE_DIM> _x_sp;
	// Vector<float, CONTROL_DIM> _u_sp;
};


#endif /* LQR_H_ */
