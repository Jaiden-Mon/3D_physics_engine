#ifndef PHY2_WORLD_HPP
#define PHY2_WORLD_HPP
#include <vector>
#include "rigidbody.hpp"

class World {
    std::vector<RigidBody*> bodies;
public:

    void add_body(RigidBody* body){bodies.push_back(body);};
    void step(float dt);
    void integrate(float dt);
    void detectCollision();
    void resolveCollision();
};


#endif //PHY2_WORLD_HPP