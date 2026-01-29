#ifndef PHY2_VECTOR3_H
#define PHY2_VECTOR3_H

#include <cmath>

struct Vector3 { //this struct defines the 3D vectors that I'm gonna use for the physics engine
    float x, y, z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {} //default constructor

    Vector3(float x,float y,float z) : x(x), y(y), z(z) {}//constructor w individual components

    explicit Vector3(float scalar) : x(scalar), y(scalar), z(scalar) {} //scalar constructor (all same)

    Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {} //copy constructor

    ~Vector3() = default; //destructor


    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }//vector addition

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }//vector substraction

    Vector3 operator*(const float scalar) const {
        return Vector3(x*scalar,y*scalar,z*scalar);
    }//vector multiplication

    Vector3 operator/(const float scalar) const {
        return Vector3(x/scalar,y/scalar,z/scalar);
    }//vector division

    Vector3& operator+=(const Vector3& other)  {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }//VVVVVV
    Vector3& operator-=(const Vector3& other)  {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }//compound add and sub

    Vector3& operator*=(const float s)  {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }//VVVVVV
    Vector3& operator/=(const float s) {
        const float inv = 1.0f / s;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }//compound mult and div

    float dot_prod(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }//scalar product of vectors, returns a float value

    Vector3 cross_prod(const Vector3& other) const {
        return Vector3(y * other.z -other.y * z,x * other.z -other.x * z,x * other.y -other.x * y);
    }//vectorial product, returns a vector

    float length() const {
        return sqrt(x*x + y*y + z*z);
    }//calculates the length of a vector, returns a float value

    float length_squared() const {
        return x*x + y*y + z*z;
    }//faster for comparison

    void normalize() {
        float mag = length();
        if (mag > 0.0f) {  // Avoid div by zero
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }//normalization function

    Vector3 normalized() const {
        Vector3 norm = *this;
        norm.normalize();
        return norm;
    } //returns the normalized length of a vector


};


#endif //PHY2_VECTOR3_H