#ifndef PHY2_QUATERNION_HPP
#define PHY2_QUATERNION_HPP
#include "matrix3x3.hpp"

#include <cmath>

struct Quaternion {
    float w= 1.0f;//scalar part
    Vector3 v = Vector3(0, 0, 0);//vector part

    Quaternion() {};//default constructor w no rotation

    Quaternion(float x, float y, float z, float w) : w(w), v(x,y,z) {}//constructor via components (where v=(x,y,z))

    static Quaternion identity() { return Quaternion(Vector3(), 1.0f); } //identity constructor for physics

    Quaternion(const Vector3& axis, float theta) {
        Vector3 nAxis = axis.normalized();
        float halfTheta = theta * 0.5f;
        w = cosf(halfTheta);
        v = nAxis * sinf(halfTheta);
    }//Axis-Angle construction, rotate theta radians around normalized axis

    Quaternion(const Matrix3x3& mat);//constructor via Matrix3x3

    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(other.v + other.v, w + other.w);
    }//addition of quaternions
    Quaternion operator-(const Quaternion& other) const {
        return Quaternion(v - other.v, w - other.w);
    }//substraction of quaternions

    Quaternion operator*(const Quaternion& rhs) const {
        float wp = w* rhs.w -v.dot_prod(rhs.v);//the p's here mean prime, like f(x) and f'(x)

        Vector3 vp = v * rhs.w + rhs.v*w  + v.cross_prod(rhs.v);
        return Quaternion(vp, wp);
    }//multiplication of quaternions; "rhs" means right-hand side

    Quaternion rotate(const Vector3& vec) const {
        Quaternion vecQ = Quaternion (vec, 0.0f);
        Quaternion result = *this * vecQ*conjugate();
        return result;
    }//rotate a 3D vector by a quaternion

    Matrix3x3 toMatrix3x3() const {
        float xx = v.x * v.x, xy = v.x * v.y, xz = v.x * v.z;
        float yy = v.y * v.y, yz = v.y * v.z, zz = v.z * v.z;
        float wx = w * v.x, wy = w * v.y, wz = w * v.z;

        // Extract right, up, forward basis vectors as columns
        Vector3 right(1-2*(yy+zz), 2*(xy+wz),   2*(xz-wy));
        Vector3 up(   2*(xy-wz),   1-2*(xx+zz), 2*(yz+wx));
        Vector3 forward(  2*(xz+wy), 2*(yz-wx),   1-2*(xx+yy));

        return Matrix3x3(right, up, forward);
    }//converts a quaternion into a matrix


    float length() const {
        return sqrtf(w*w+v.length_squared());
    }//pretty obvious what ts does

    void normalize() {
        float mag = length();
        if (mag > 0.0f) {  // Avoid div by zero
            v /= mag;
            w /= mag;
        }
    }                //VVVVVV
    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }   //handle normalization

    float dot(const Quaternion& other) const {
        return w*other.w + v.dot_prod(other.v);
    }//ts does the dot product of 2 Quaternions

    Quaternion conjugate() const {
        return Quaternion(v-v-v, w);
    }//ts does the conjugate of a quaternion



};


#endif //PHY2_QUATERNION_HPP
