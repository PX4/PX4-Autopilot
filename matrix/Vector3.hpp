/**
 * @file Vector3.hpp
 *
 * 3D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Vector.hpp>

namespace matrix
{

template<typename Type>
class Vector3 : public Vector<Type, 3>
{
public:
    virtual ~Vector3() {};

    Vector3() : Vector<Type, 3>()
    {
    }

    Vector3(Type x, Type y, Type z) : Vector<Type, 3>()
    {
        Vector3 &v(*this);
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    Vector3 cross(const Vector3 & b) {
        // TODO
        Vector3 &a(*this);
        (void)a;
        Vector3 c;
        c(0) = 0;
        c(1) = 0;
        c(2) = 0;
        return c;
    }
};

typedef Vector3<float> Vector3f;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
