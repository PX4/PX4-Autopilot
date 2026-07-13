// Out-of-line definition of matrix::wrap(float) (see helper_functions.hpp);
// kept out of line to save flash, as it would otherwise be inlined into
// hundreds of call sites via wrap_pi()/wrap_2pi().
#include "math.hpp"

namespace matrix
{

float wrap(float x, float low, float high)
{
	return matrix::detail::wrap_floating(x, low, high);
}

} // namespace matrix
