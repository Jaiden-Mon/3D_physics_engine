cmake_minimum_required(VERSION 4.1)
project(phy2)

set(CMAKE_CXX_STANDARD 26)

add_executable(phy2 main.cpp
        vector3.hpp
        matrix3x3.hpp
        quaternion.hpp
        rigidbody.hpp
        world.cpp
        world.hpp
        collision.hpp
)
