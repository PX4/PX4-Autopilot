#include "Vector.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	Vector<float, 5> v;
	float n = v.norm();
	float r = v.dot(v);
	return 0;
}
