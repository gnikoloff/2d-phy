#ifndef BODY_H
#define BODY_H

#include "../math/Vec2.h"
#include "../rigidbody/Shape.h"

class Body {
public:
  // Linear motion
  Vec2 position;
  Vec2 velocity;
  Vec2 acceleration;

  // Angular motion
  float rotation;
  float angularVelocity;
  float angularAcceleration;

  // Forces and torque
  Vec2 sumForces;
  float sumTorque;

  // Mass and Moment of Inertia
  float mass;
  float invMass;
  float I;
  float invI;

  // Coefficient of restitution (elasticity)
  float restitution;

  // Coefficient of friction
  float friction;

  float boundingCircleRadius;

  // Pointer to the shape/geometry of this rigid body
  Shape* shape = nullptr;

  Body(const Shape& shape, float x, float y, float mass);
  Body(float radius, float x, float y, float mass);
  Body(float width, float height, float x, float y, float mass);
  Body(const std::vector<Vec2> vertices, float x, float y, float mass, float empty0, float empty1);
  ~Body();

  Vec2 GetPosition() const;
  void SetPosition(const Vec2& v);
  Vec2 GetVelocity() const;
  void SetVelocity(const Vec2& v);
  Vec2 GetAcceleration() const;
  void SetAcceleration(const Vec2& v);

  float GetRotation() const;
  void SetRotation(const float rotation);
  float GetAngularVelocity() const;
  void SetAngularVelocity(const float angularVelocity);
  float GetAngularAcceleration() const;
  void SetAngularAcceleration(const float angularAcceleration);

  Vec2 GetSumForces() const;
  void SetSumForces(const Vec2& sumForces);
  float GetSumTorque() const;
  void SetSumTorque(const float sumTorque);

  float GetMass() const;
  void SetMass(const float mass);
  float GetInvMass() const;
  void SetInvMass(const float invMass);
  float GetI() const;
  void SetI(const float I);
  float GetInvI() const;
  void SetInvI(const float I);

  float GetRestitution() const;
  void SetRestitution(const float restitution);
  float GetFriction() const;
  void SetFriction(const float friction);

  float GetBoundingCircleRadius() const;

  bool IsStatic() const;

  void AddForce(const Vec2& force);
  void AddTorque(float torque);
  void ClearForces();
  void ClearTorque();

  Vec2 LocalSpaceToWorldSpace(const Vec2& point) const;
  Vec2 WorldSpaceToLocalSpace(const Vec2& point) const;

  void ApplyImpulseLinear(const Vec2& j);
  void ApplyImpulseAngular(const float j);
  void ApplyImpulseAtPoint(const Vec2& j, const Vec2& r);

  void IntegrateLinear(float dt);
  void IntegrateAngular(float dt);

  void IntegrateForces(const float dt);
  void IntegrateVelocities(const float dt);
};

#endif
