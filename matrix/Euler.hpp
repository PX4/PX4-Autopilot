/**
 * @file Euler.hpp
 *
 * All rotations and axis systems follow the right-hand rule
 *
 * An instance of this class defines a rotation from coordinate frame 1 to coordinate frame 2.
 * It follows the convention of an intrinsic tait-bryan 3-2-1 sequence.
 * In order to go from frame 1 to frame 2 we apply the following rotations consecutively.
 * 1) We rotate about our initial Z axis by an angle of _psi.
 * 2) We rotate about the newly created Y' axis by an angle of _theta.
 * 3) We rotate about the newly created X'' axis by an angle of _phi.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

/**
 * Euler angles class
 *
 * This class describes the transformation from the body fixed frame
 * to the inertial frame via 3-2-1 tait brian euler angles.
 */
template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
    virtual ~Euler() {};

    /**
     * Standard constructor
     */
    Euler() : Vector<Type, 3>()
    {
    }

    /**
     * Copy constructor
     *
     * @param other vector to copy
     */
    Euler(const Vector<Type, 3> & other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from Matrix31
     *
     * @param other Matrix31 to copy
     */
    Euler(const Matrix<Type, 3, 1> & other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from euler angles
     *
     * Instance is initialized from angle tripplet (3,2,1)
     * representing transformation from body frame to inertial frame. 
     *
     * @param phi_ rotation angle about X axis
     * @param theta_ rotation angle about Y axis
     * @param psi_ rotation angle about Z axis
     */
    Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        set_from_euler(phi_, theta_, psi_);
    }

    /**
     * Constructor from DCM matrix
     *
     * Instance is set from Dcm representing
     * transformation from frame 2 to frame 1.
     * This instance will hold the angles defining the rotation
     * from frame 1 to frame 2.
     *
     * @param dcm Direction cosine matrix
    */
    Euler(const Dcm<Type> & dcm) : Vector<Type, 3>()
    {
        set_from_dcm(dcm);
    }

    /**
     * Constructor from quaternion instance.
     *
     * Instance is set from a quaternion representing
     * transformation from frame 2 to frame 1.
     * This instance will hold the angles defining the rotation
     * from frame 1 to frame 2.
     *
     * @param q quaternion
    */
    Euler(const Quaternion<Type> & q) :
        Vector<Type, 3>()
    {
        set_from_quaternion(q);
    }

    /**
     * Set from euler angles
     *
     * Instance is set from angle tripplet (3,2,1) representing
     * rotation from frame 1 to frame 2.
     *
     * @param phi_   roll
     * @param theta_ pitch
     * @param psi_   yaw
     */
    void set_from_euler(Type phi_, Type theta_, Type psi_)
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    /**
     * Set from dcm
     *
     * Instance is set from dcm representing transformation
     * from frame 2 to frame 1.
     *
     * @param dcm Direction cosine matrix instance to convert from.
     */
    void set_from_dcm(const Dcm<Type> & dcm)
    {
        Type phi_val = Type(atan2(dcm(2,1), dcm(2,2)));
        Type theta_val = Type(asin(-dcm(2,0)));
        Type psi_val = Type(atan2(dcm(1,0), dcm(0,0)));
        Type pi = Type(M_PI);

        if (fabs(theta_val - pi/2) < 1.0e-3) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(dcm(1,2), dcm(0,2)));
        } else if (Type(fabs(theta_val + pi/2)) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(-dcm(1,2), -dcm(0,2)));
        }

        phi() = phi_val;
        theta() = theta_val;
        psi() = psi_val;
    }

    /**
     * Set from dcm
     *
     * Instance is set from quaternion representing
     * transformation from body frame to inertial frame.
     *
     * @param q quaternion to set angles to
     */
    void set_from_quaternion(const Quaternion<Type> & q)
    {
        *this = Euler(Dcm<Type>(q));
    }

    inline Type phi() const {
        return (*this)(0);
    }
    inline Type theta() const {
        return (*this)(1);
    }
    inline Type psi() const {
        return (*this)(2);
    }

    inline Type & phi() {
        return (*this)(0);
    }
    inline Type & theta() {
        return (*this)(1);
    }
    inline Type & psi() {
        return (*this)(2);
    }

};

typedef Euler<float> Eulerf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
