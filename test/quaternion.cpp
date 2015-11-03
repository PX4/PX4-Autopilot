#include "Quaternion.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	Quatf p(1, 0, 0, 0);
	Quatf q(0, 1, 0, 0);
	Quatf r = p*q;
	Dcmf dcm = Dcmf(p);
	Eulerf e = Eulerf(p);
	return 0;
}
