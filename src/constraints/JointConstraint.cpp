#include "JointConstraint.h"

JointConstraint::JointConstraint() : Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
  cachedLambda.Zero();
}

JointConstraint::JointConstraint(Body* a, Body* b, const Vec2& anchorPoint) : Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
  this->a = a;
  this->b = b;
  this->aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
  this->bPoint = b->WorldSpaceToLocalSpace(anchorPoint);
  cachedLambda.Zero();
}

Body* JointConstraint::GetBody(const int idx) {
  if (idx == 0) {
    return this->a;
  }
  return this->b;
}

void JointConstraint::PreSolve(const float dt) {
  // Get the anchor point position in world space
  const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
  const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);

  const Vec2 ra = pa - a->position;
  const Vec2 rb = pb - b->position;

  jacobian.Zero();

  Vec2 J1 = (pa - pb) * 2.0;
  jacobian.rows[0][0] = J1.x; // A linear velocity.x
  jacobian.rows[0][1] = J1.y; // A linear velocity.y

  float J2 = ra.Cross(pa - pb) * 2.0;
  jacobian.rows[0][2] = J2;   // A angular velocity

  Vec2 J3 = (pb - pa) * 2.0;
  jacobian.rows[0][3] = J3.x; // B linear velocity.x
  jacobian.rows[0][4] = J3.y; // B linear velocity.y

  float J4 = rb.Cross(pb - pa) * 2.0;
  jacobian.rows[0][5] = J4;   // B angular velocity

  // Warm starting (apply cached lambda)
  const MatMN Jt = jacobian.Transpose();
  VecN impulses = Jt * cachedLambda;

  // Apply the impulses to both bodies 
  a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
  a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
  b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
  b->ApplyImpulseAngular(impulses[5]);                   // B angular impulse

  // Compute the bias term (baumgarte stabilization)
  const float beta = 0.5f;
  float C = (pb - pa).Dot(pb - pa);
  C = std::max(0.0f, C - 0.01f);
  bias = (beta / dt) * C;
}

void JointConstraint::Solve() {
  const VecN V = GetVelocities();
  const MatMN invM = GetInvM();

  const MatMN J = jacobian;
  const MatMN Jt = jacobian.Transpose();

  // Compute lambda using Ax=b (Gauss-Seidel method) 
  MatMN lhs = J * invM * Jt;  // A
  VecN rhs = J * V * -1.0f;   // b
  rhs[0] -= bias;
  VecN lambda = MatMN::SolveGaussSeidel(lhs, rhs);
  cachedLambda += lambda;

  // Compute the impulses with both direction and magnitude
  VecN impulses = Jt * lambda;

  // Apply the impulses to both bodies 
  a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1])); // A linear impulse
  a->ApplyImpulseAngular(impulses[2]);                   // A angular impulse
  b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4])); // B linear impulse
  b->ApplyImpulseAngular(impulses[5]);                   // B angular impulse
}

void JointConstraint::PostSolve() {
  // TODO: Maybe we should clamp the values of cached lambda to reasonable limits
}