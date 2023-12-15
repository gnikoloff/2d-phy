#ifndef PENETRATION_CONSTRAINT_H
#define PENETRATION_CONSTRAINT_H

#include "Constraint.h"

class PenetrationConstraint : public Constraint {
  private:
    MatMN jacobian;
    VecN cachedLambda;
    float bias;
    Vec2 normal;
    float friction;

  public:
    PenetrationConstraint();
    PenetrationConstraint(Body* a, Body* b, const Vec2& aCollisionPoint, const Vec2& bCollisionPoint, const Vec2& normal);
    void PreSolve(const float dt) override;
    void Solve() override;
    void PostSolve() override;
};

#endif