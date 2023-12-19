#ifndef JOINT_CONSTRAINT_H
#define JOINT_CONSTRAINT_H

#include "Constraint.h"

class JointConstraint : public Constraint {
private:
  MatMN jacobian;
  VecN cachedLambda;
  float bias;

public:
  JointConstraint();
  JointConstraint(Body* a, Body* b, const Vec2& anchorPoint);
  Body* GetBody(const int idx);
  void PreSolve(const float dt) override;
  void Solve() override;
  void PostSolve() override;
};

#endif
