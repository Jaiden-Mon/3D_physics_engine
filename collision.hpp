#ifndef PHY2_COLLISION_HPP
#define PHY2_COLLISION_HPP
#include <vector>
#include "vector3.hpp"
#include "rigidbody.hpp"

struct Collisions {
    static std::vector<Collisions> collisions;

    RigidBody *bodyA, *bodyB;
    Vector3 normal;
    Vector3 contact_p;
    float penetration; //Lol

};

inline bool check_StS(RigidBody* a, RigidBody* b, Collisions& contact) {
    Vector3 delta = b->position - a->position;
    float distance = delta.length();
    float collisionDistance = a->sphere.radius + b->sphere.radius;  // =1.0

    if (distance >= collisionDistance || distance == 0) return false;

    contact.normal = delta.normalized();
    contact.penetration = collisionDistance - distance;
    contact.bodyA = a;
    contact.bodyB = b;
    return true;
}


#endif //PHY2_COLLISION_HPP