#include "World.h"
#include "Constants.h"
#include "collisions/CollisionDetection.h"
#include "constraints/PenetrationConstraint.h"
#include "constraints/JointConstraint.h"
#include <iostream>

World::World(float gravity, float width, float height) {
  G = -gravity;
  this->width = width;
  this->height = height;
}

World::~World() {
  for (auto body : bodies) {
    delete body;
  }
  for (auto constraint : constraints) {
    delete constraint;
  }
}

void World::AddBody(Body* body) {
  bodies.push_back(body);
}

std::vector<Body*>& World::GetBodies() {
  return bodies;
}

Body* World::GetBody(const int idx) {
  return bodies[idx];
}

void World::AddJointConstraint(JointConstraint* constraint) {
  constraints.push_back(constraint);
}

std::vector<Constraint*>& World::GetConstraints() {
  return constraints;
}

void World::AddForce(const Vec2& force) {
  forces.push_back(force);
}

void World::AddTorque(float torque) {
  torques.push_back(torque);
}

void World::Update(float dt) {
  // Create a vector of penetration constraints that will be solved frame per frame
  std::vector<PenetrationConstraint> penetrations;

  // Loop all bodies of the world applying forces
  for (auto& body : bodies) {
    // Apply the weight force to all bodies
    Vec2 weight = Vec2(0.0, body->mass * G * PIXELS_PER_METER);
    body->AddForce(weight);

    // Apply forces to all bodies
    for (auto force : forces)
      body->AddForce(force);

    // Apply torque to all bodies
    for (auto torque : torques)
      body->AddTorque(torque);
  }

  // Integrate all the forces
  for (auto& body : bodies) {
    body->IntegrateForces(dt);
  }

  for (int i = 0; i <= bodies.size() - 1; i++) {
    for (int j = i + 1; j < bodies.size(); j++) {
      Body* a = bodies[i];
      Body* b = bodies[j];

      // broadphase
      if (
        (a->position.x + a->boundingCircleRadius < width * 0.5 && b->position.x - b->boundingCircleRadius > width * 0.5) ||
        (b->position.x + b->boundingCircleRadius < width * 0.5 && a->position.x - a->boundingCircleRadius > width * 0.5) ||
        (a->position.y + a->boundingCircleRadius < height * 0.5 && b->position.y - b->boundingCircleRadius > height * 0.5) ||
        (b->position.y + b->boundingCircleRadius < height * 0.5 && a->position.y - a->boundingCircleRadius > height * 0.5)) {
        continue;
      }

      if (a->shape->GetType() != CIRCLE || b->shape->GetType() != CIRCLE) {
        const Vec2 ab = b->position - a->position;
        const float radiusSum = a->boundingCircleRadius + b->boundingCircleRadius;
        if (ab.MagnitudeSquared() > (radiusSum * radiusSum)) {
          continue;
        }
      }

      // narrowphase
      std::vector<Contact> contacts;
      if (CollisionDetection::IsColliding(a, b, contacts)) {
        for (auto contact : contacts) {
          // Create a new penetration constraint
          PenetrationConstraint penetration(contact.a, contact.b, contact.start, contact.end, contact.normal);
          penetrations.push_back(penetration);
        }
      }
    }
  }

  // Solve all constraints
  for (auto& constraint : constraints) {
    constraint->PreSolve(dt);
  }
  for (auto& constraint : penetrations) {
    constraint.PreSolve(dt);
  }
  for (int i = 0; i < 4; i++) {
    for (auto& constraint : constraints) {
      constraint->Solve();
    }
    for (auto& constraint : penetrations) {
      constraint.Solve();
    }
  }
  for (auto& constraint : constraints) {
    constraint->PostSolve();
  }
  for (auto& constraint : penetrations) {
    constraint.PostSolve();
  }

  // Integrate all the velocities
  for (auto& body : bodies) {
    body->IntegrateVelocities(dt);
  }
}
