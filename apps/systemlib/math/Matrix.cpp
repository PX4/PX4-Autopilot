#include "Matrix.h"

namespace math
{

int testMatrix()
{
    const float data_a[] = 
        {1,2,
         3,4};
    const float data_b[] = 
        {5,6,
         7,8};
    const float data_c[] = 
        {1,2};
    MatrixFloat a(2,2,data_a);
    MatrixFloat b(2,2,data_b);
    VectorFloat c(2,data_c);

    a.print();
    b.print();
    c.print();

    return 0;
}

} // namespace math
