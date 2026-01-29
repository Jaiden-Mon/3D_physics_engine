#include <iostream>
#include <iomanip>
#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "world.hpp"
#include "rigidbody.hpp"



int main() {
    World world;

    RigidBody* sphere= new RigidBody(1.0f);//a sphere w mass 1kg
    sphere-> position = Vector3(0.0f, 5.0f, 0.0f);//for it to start at 3m high
    sphere-> inv_inertia_l = Matrix3x3();//simplified sphere-like
    world.add_body(sphere); //adds the sphere into the world

    RigidBody* ground =  new RigidBody(0.0f);//creates a static ground w infinite mass
    ground-> position = Vector3(0.0f, 0.0f, 0.0f);
    world.add_body(ground);//adds the ground into the world,for later collisions

    std::cout<<std::fixed<<std::setprecision(2);
    std::cout << "Time\tPosY\t\t VelY\n";

    float time = 0.0f;
    for (int step = 0; step < 600; step++) {
        world.step(1.0f/60.0f);
        time += 1.0f/60.0f;

        if (step % 30 == 0) {
            std::cout << time << "\t"<< sphere->position.y << "\t \t"<< sphere->l_velocity.y << "\n";
        }//prints every 0.5 seconds

    }

    delete sphere;
    delete ground;
    return 0;
}