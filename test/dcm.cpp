#include "Dcm.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	Dcmf dcm;
	Quatf q = Quatf(dcm);
	Eulerf e = Eulerf(dcm);
	return 0;
}
