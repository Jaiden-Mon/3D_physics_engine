#include "world.hpp"
#include "rigidbody.hpp"
#include "vector3.hpp"
#include "matrix3x3.hpp"
#include "quaternion.hpp"
#include <cmath>

void World::step(float dt) {
    static float acc = 0.0f;//fixed timestep for stability
    const float fixed_dt = 1.0f / 60.0f;//60fps

    acc += dt;

    while (acc >= fixed_dt) {
        integrate(fixed_dt);
        acc -= fixed_dt;
    }

}

void World::integrate(float dt) {

        for (RigidBody* body : bodies) {
            if (body->inv_mass == 0.0f) continue; //so it skips static bodies

            body->force_acc += Vector3(0.0f, 9.81f, 0.0f)*body->mass;//applies gravity

            //for linear integration VVVVVV
            Vector3 acceleration = body->force_acc * body->inv_mass;
            body->l_velocity += acceleration * dt;
            body->position += body->l_velocity * dt;

            //for transforming angular velocity and torque into body space VVVVVV
            Matrix3x3 rot_m = body->orientation.toMatrix3x3();
            Vector3 body_a_velocity = rot_m.transposed() * body->a_velocity;
            Vector3 body_torque =rot_m.transposed() * body->torque_acc;

            //for angular integration VVVVVV
            Vector3 angular_accel = body->inv_inertia_l*body_torque;
            body->a_velocity += angular_accel * dt;

            //for orientation integration (angular velocity -> quat derivative) VVVVVV
            Quaternion delta_quat(body->a_velocity*(0.5f*dt),0.0f);
            body->orientation =(body->orientation * delta_quat).normalized();

            //for applying damping VVVVVV
            body->l_velocity *= 0.995f;
            body->a_velocity *= 0.995f;

            body->force_acc = Vector3();
            body->torque_acc = Vector3();


        }
}