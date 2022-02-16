#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	Euler<float> euler(0.1f, 0.2f, 0.3f);
	Dcm<float> R(euler);
	Dcm<float> skew = R - R.T();
	Vector3<float> w = skew.vee();
	Vector3<float> w_check(0.1348f, 0.4170f, 0.5647f);

	TEST(isEqual(w, w_check));
	TEST(isEqual(skew, w.hat()));
	return 0;
}

