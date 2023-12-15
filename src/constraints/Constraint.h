#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "../rigidbody/Body.h"
#include "../math/MatMN.h"

class Constraint {
  public:
    Body* a;
    Body* b;

    Vec2 aPoint; // The constraint point in A's local space
    Vec2 bPoint; // The constraint point in B's local space

    virtual ~Constraint() = default;

    MatMN GetInvM() const;
    VecN GetVelocities() const;

    virtual void PreSolve(const float dt) {}
    virtual void Solve() {}
    virtual void PostSolve() {}
};

#endif
