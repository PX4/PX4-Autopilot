#include "Matrix.h"

namespace math
{

int testMatrix()
{
    MatrixFloat a(2,2);
    const float data[] = {1,0,0,1};
    a.set(data);
    MatrixFloat b(2,2);
    MatrixFloat c(2,2);
    c = a+b;
    c.print();
    c = a-b;
    c.print();
    c = a*b;
    c.print();
    c = a/b;
    c.print();
    return 0;
}

} // namespace math
