#include <iostream>
#include "Vector3.hpp"
#include "Matrix3x3.hpp"

using namespace std;


int main() {

    Vector3 u = Vector3(1,2,3);
    Vector3 v = Vector3(4,5,6);
    Vector3 w = Vector3(7,8,9);

    Matrix3x3 A = Matrix3x3(u,v,w);
    Matrix3x3 B = Matrix3x3(v,w,u);
    Matrix3x3 C = B*A;

    for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
        cout<<C(row,col)<<" ";
    }
    cout<<endl;
    }


    return 0;
}

/*for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
        cout<<A(row,col)<<" ";
    }
    cout<<endl;
}
*/