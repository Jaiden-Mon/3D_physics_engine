#include <iostream>
#include <iomanip>
#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "world.hpp"
#include "rigidbody.hpp"
#include "collision.hpp"


int main() {
    World world;

    //sphere A, above
    auto* A = new RigidBody(1.0f, 0.5f);
    A->position = Vector3(0,1.2f,0);
    A->l_velocity = Vector3(0,-2.0,0); //falling
    world.add_body(A);

    //sphere B, below
    auto* B = new RigidBody(1.0f, 0.5f);
    B->position = Vector3(0, 0.0f, 0);
    B->l_velocity = Vector3(0, 0.0f, 0);
    world.add_body(B);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "t\t\t Ay\t\t VyA \t\t By \t\t VyB \n";

    float t = 0.0f;
    const float dt = 1.0f / 60.0f;

    for (int i = 0; i < 240; i++) {
        world.step(dt);
        t += dt;

        if (i % 10 == 0) { // print every ~0.16s
            std::cout << t << "\t"
                      << A->position.y << "\t\t" << A->l_velocity.y << "\t\t"
                      << B->position.y << "\t\t" << B->l_velocity.y << "\n";


        }
    }

    delete A;
    delete B;

}