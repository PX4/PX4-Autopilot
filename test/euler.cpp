#include "Euler.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	Eulerf e;
	float dp = e.T()*e;
	Dcmf dcm = Dcmf(e);
	Quatf q = Quatf(e);
	float n = e.norm();
	return 0;
}
