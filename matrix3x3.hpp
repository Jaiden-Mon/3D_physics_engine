#ifndef PHY2_MATRIX3X3_HPP
#define PHY2_MATRIX3X3_HPP
#include <array>
#include "Vector3.hpp"

struct Matrix3x3 {

    std::array<float,9> m ={1,0,0,
                            0,1,0,
                            0,0,1};//identity in constructor

    Matrix3x3()= default;//already set above, but anyways

    Matrix3x3(float sx,float sy,float sz) {
        m[0] =sx; m[4]=sy; m[8]=sz;
    }//Initial constructor

    Matrix3x3(const Vector3& x, const Vector3& y, const Vector3& z) {
        m[0]=x.x; m[3]=x.y; m[6]=x.z;
        m[1]=y.x; m[4]=y.y; m[7]=y.z;
        m[2]=z.x; m[5]=z.y; m[8]=z.z;
    } //for rotations around an axis (w quaternions later)

    Matrix3x3 operator+(const Matrix3x3& other) const {
        Matrix3x3 result;
        for (int i=0;i<9;i++) {
            result.m[i]=m[i]+other.m[i];
        }
        return result;
    }//adds 2 matrices together

    Matrix3x3 operator-(const Matrix3x3& other) const {
        Matrix3x3 result;
        for (int i=0;i<9;i++) {
            result.m[i]=m[i]-other.m[i];
        }
        return result;
    }//subtracts 2 matrices

    Matrix3x3 operator*(const float scalar) const {
        Matrix3x3 result;
        for (int i=0;i<9;i++) {
            result.m[i]=m[i]*scalar;
        }
        return result;
    }//multiplies a matrix times a scalar

    Matrix3x3 operator/(const float scalar) const {
        Matrix3x3 result;
        for (int i=0;i<9;i++) {
            result.m[i]=m[i]/scalar;
        }
        return result;
    }//division of a matrix by a scalar

    Vector3 operator*(const Vector3& v) const {
        return Vector3(
            m[0]*v.x + m[3]*v.y + m[6]*v.z,
            m[1]*v.x + m[4]*v.y + m[7]*v.z,
            m[2]*v.x + m[5]*v.y + m[8]*v.z//ts basically multiplies a vector per every column
        );
    }//multiplies a matrix and a vector,returning a vector

    Matrix3x3 operator*(const Matrix3x3& other) {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i*3 + j] = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    result.m[i*3 + j] += m[i*3 + k] * other.m[k*3 + j];
                }
            }
        }
        return result;
    }//multiplication of two matrices!1!!!

    void transpose() {
        std::swap(m[1], m[3]);
        std::swap(m[2], m[6]);
        std::swap(m[5], m[7]);
    }               //VVVVVVV
    Matrix3x3 transposed() const {
        Matrix3x3 result = *this;
        result.transpose();
        return result;
    }   //ts two swap rows and columns

    float determinant() const {
        return   m[0]*(m[4]*m[8]-m[5]*m[7])
                -m[1]*(m[3]*m[8]-m[5]*m[6])
                +m[2]*(m[3]*m[7]-m[4]*m[6]);
    }//for finding the determinant of a given matrix

    Matrix3x3 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) { /* handle singular */ }

        // Adjugate elements (cofactors transposed)
        float inv_det = 1.0f / det;
        Matrix3x3 result;
        result.m[0] =  (m[4]*m[8] - m[5]*m[7]) * inv_det;  // C11
        result.m[1] = -(m[1]*m[8] - m[2]*m[7]) * inv_det;  // C21
        result.m[2] =  (m[1]*m[5] - m[2]*m[4]) * inv_det;  // C31

        result.m[3] = -(m[3]*m[8] - m[5]*m[6]) * inv_det;  // C12
        result.m[4] =  (m[0]*m[8] - m[2]*m[6]) * inv_det;  // C22
        result.m[5] = -(m[0]*m[5] - m[2]*m[3]) * inv_det;  // C32

        result.m[6] =  (m[3]*m[7] - m[4]*m[6]) * inv_det;  // C13
        result.m[7] = -(m[0]*m[7] - m[1]*m[6]) * inv_det;  // C23
        result.m[8] =  (m[0]*m[4] - m[1]*m[3]) * inv_det;  // C33

        return result;
    }//inverses a matrix

    bool isOrthogonal(const Matrix3x3& m) {
        Vector3 col0(m.m[0], m.m[3], m.m[6]);
        Vector3 col1(m.m[1], m.m[4], m.m[7]);
        Vector3 col2(m.m[2], m.m[5], m.m[8]);

        return std::abs(col0.dot_prod(col1)) < 1e-6f &&  // Orthogonal
               std::abs(col0.length() - 1.0f) < 1e-6f; // Unit length
    } //checks if a matrix is orthogonal (for future things w quaternions)




    float& operator()(int row, int col) { return m[row*3 + col]; }      //VVVVVVV
    float operator()(int row, int col) const { return m[row*3 + col]; } //both for easy printing/ access


};
#endif //PHY2_MATRIX3X3_HPP