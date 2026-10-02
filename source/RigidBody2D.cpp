#include "RigidBody2D.hpp"

// Advances the body by one fixed step (semi-implicit Euler).
// dt must be > 0.
void RigidBody2D::integrate(float dt) noexcept {
    if (dt > 0.0f) {
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;
    }/*else{
       // stderr("Time step must be positive.");
    }*/
}