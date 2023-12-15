#include "Force.h"
#include <algorithm>

Vec2 Force::GenerateDragForce(const Body& body, float k) {
  Vec2 dragForce = Vec2(0, 0);
  if (body.velocity.MagnitudeSquared() > 0) {
    // drag direction (inverse of velocity unit vector)
    Vec2 dragDirection = body.velocity.UnitVector() * -1.0;

    // k * |v|^2
    float dragMagnitude = k * body.velocity.MagnitudeSquared();

    // Generate the final drag force with direction and magnitude
    dragForce = dragDirection * dragMagnitude;
  }
  return dragForce;
}

Vec2 Force::GenerateFrictionForce(const Body& body, float k) {
  // friction direction (inverse of velocity unit vector)
  Vec2 frictionDirection = body.velocity.UnitVector() * -1.0;
  float frictionMagnitude = k;
  Vec2 frictionForce = frictionDirection * frictionMagnitude;
  return frictionForce;
}

Vec2 Force::GenerateGravitationalForce(const Body& a, const Body& b, float G, float minDistance, float maxDistance) {
  Vec2 d = (b.position - a.position);
  float distanceSquared = d.MagnitudeSquared();
  distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);
  Vec2 attractionDirection = d.UnitVector();
  float attractionMagnitude = G * (a.mass * b.mass) / distanceSquared;
  Vec2 attractionForce = attractionDirection * attractionMagnitude;
  return attractionForce;
}

Vec2 Force::GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k) {
  Vec2 d = body.position - anchor;
  float displacement = d.Magnitude() - restLength;
  Vec2 springDirection = d.UnitVector();
  float springMagnitude = -k * displacement;
  Vec2 springForce = springDirection * springMagnitude;
  return springForce;
}
