#ifndef WORLD_H
#define WORLD_H

#include "rigidbody/Body.h"
#include "constraints/Constraint.h"
#include "constraints/JointConstraint.h"
#include <vector>

class World {
private:
  float width;
  float height;
  float G = 9.8;
  std::vector<Body*> bodies;
  std::vector<Constraint*> constraints;

  std::vector<Vec2> forces;
  std::vector<float> torques;

public:
  World(float gravity, float width, float height);
  ~World();

  void AddBody(Body* body);
  std::vector<Body*>& GetBodies();
  Body* GetBody(const int idx);

  void AddJointConstraint(JointConstraint* constraint);
  std::vector<Constraint*>& GetConstraints();

  float GetGravity() const;
  void SetGravity(const float gravity);

  void AddForce(const Vec2& force);
  void AddTorque(float torque);

  void Update(float dt);

  void CheckCollisions();
};

#endif
