#include "../../../../../matrix/matrix/math.hpp"

typedef matrix::SquareMatrix<float, 24> SquareMatrix24f;

inline float sq(float in) {
    return in * in;
}

namespace ecl{
	inline float powf(float x, int exp)
	{
		float ret;
		if (exp > 0) {
			ret = x;
			for (int count = 1; count < exp; count++) {
				ret *= x;
			}
			return ret;
		} else if (exp < 0) {
			return 1.0f / ecl::powf(x, -exp);
		}
		return 1.0f;
	}
}
