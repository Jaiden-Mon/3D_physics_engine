#ifndef PHY2_RIGID_BODY_HPP
#define PHY2_RIGID_BODY_HPP

#include "vector3.hpp"
#include "matrix3x3.hpp"
#include "quaternion.hpp"

struct RigidBody {

    Vector3 position;
    Quaternion orientation;
    Vector3 l_velocity;
    Vector3 a_velocity;

    float mass;
    float inv_mass;

    Matrix3x3 inv_inertia_l;//Body-fixed inverse inertia tensor

    Vector3 force_acc;
    Vector3 torque_acc;

    Vector3 prev_position;
    Quaternion prev_orientation;

    RigidBody(float m): mass(m), inv_mass(m > 0 ? 1.0f/m : 0.0f){};

};

#endif //PHY2_RIGID_BODY_HPP