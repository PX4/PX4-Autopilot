#include "Vector.h"

namespace math
{

int testVector()
{
    using namespace math;

    const float data_a[] =
        {1,2};
    const float data_b[] =
        {3,4};

    VectorFloat a(2,data_a);
    VectorFloat b(2,data_b);

    a.print();
    b.print();

    return 0;
}

} // namespace math
