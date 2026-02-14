#include "world.hpp"
#include "rigidbody.hpp"
#include "vector3.hpp"
#include "matrix3x3.hpp"
#include "quaternion.hpp"
#include <cmath>
#include <iostream>

#include "collision.hpp"
#include <vector>

std::vector<Collisions> Collisions::collisions;

void World::step(float dt) {
    static float acc = 0.0f;
    const float fixed_dt = 1.0f / 60.0f;

    acc += dt;

    while (acc >= fixed_dt) {
        detectCollision();
        resolveCollision();
        integrate(fixed_dt);
        acc -= fixed_dt;
    }
}

void World::integrate(float dt) {
    for (RigidBody* body : bodies) {
        if (body->inv_mass == 0.0f) continue;

        body->force_acc += Vector3(0.0f, -9.81f, 0.0f)*body->mass;

        Vector3 acceleration = body->force_acc * body->inv_mass;
        body->l_velocity += acceleration * dt;
        body->position += body->l_velocity * dt;

        Matrix3x3 rot_m = body->orientation.toMatrix3x3();
        Vector3 body_a_velocity = rot_m.transposed() * body->a_velocity;
        Vector3 body_torque =rot_m.transposed() * body->torque_acc;

        Vector3 angular_accel = body->inv_inertia_l*body_torque;
        body->a_velocity += angular_accel * dt;

        Quaternion delta_quat(body->a_velocity*(0.5f*dt),0.0f);
        body->orientation =(body->orientation * delta_quat).normalized();

        body->l_velocity *= 0.995f;
        body->a_velocity *= 0.995f;

        body->force_acc = Vector3();
        body->torque_acc = Vector3();
    }
}

void World::detectCollision() {
    Collisions::collisions.clear();
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i+1; j < bodies.size(); j++) {
            if (bodies[i]->inv_mass == 0 && bodies[j]->inv_mass == 0) continue;

            Collisions contact;
            if (check_StS(bodies[i], bodies[j], contact)) {
                std::cout << "collision, dist=" << (bodies[i]->position - bodies[j]->position).length() << "\\n";
                Collisions::collisions.push_back(contact);
            }
        }
    }
}

void World::resolveCollision() {
    for (Collisions& collision : Collisions::collisions) {
        RigidBody* a = collision.bodyA;
        RigidBody* b = collision.bodyB;

        Vector3 rel_velocity = b->l_velocity - a->l_velocity;
        float vel_along_normal = rel_velocity.dot_prod(collision.normal);

        if (vel_along_normal > 0.0f) continue;

        float inv_mass_sum = a->inv_mass + b->inv_mass;
        float j = -(1.0f + 0.5f) * vel_along_normal / inv_mass_sum;

        Vector3 impulse = collision.normal * j;
        a->l_velocity -= impulse * a->inv_mass;
        b->l_velocity += impulse * b->inv_mass;

        const float percent = 0.2f;
        const float slop = 0.01f;
        float correction_dist = std::max(collision.penetration - slop, 0.0f) * percent;
        Vector3 correction = collision.normal * correction_dist / inv_mass_sum;
        a->position -= correction * a->inv_mass;
        b->position += correction * b->inv_mass;
    }
}
