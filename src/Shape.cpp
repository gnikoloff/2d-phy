#include "Shape.h"
#include <iostream>
#include <limits>

CircleShape::CircleShape(float radius) {
  this->radius = radius;
  std::cout << "CircleShape constructor called!" << std::endl;
}

CircleShape::~CircleShape() {
  std::cout << "CircleShape destructor called!" << std::endl;
}

Shape* CircleShape::Clone() const {
  return new CircleShape(radius);
}

void CircleShape::UpdateVertices(float angle, const Vec2& position) {
  return; // Circles don't have vertices... nothing to do here
}

ShapeType CircleShape::GetType() const {
  return CIRCLE;
}

float CircleShape::GetMomentOfInertia() const {
  // For solid circles, the moment of inertia is 1/2 * r^2
  // But this still needs to be multiplied by the rigidbody's mass
  return 0.5 * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
  // Initialize the vertices of the polygon shape
  for (auto vertex : vertices) {
    localVertices.push_back(vertex);
    worldVertices.push_back(vertex);
  }
  std::cout << "PolygonShape constructor called!" << std::endl;
}

PolygonShape::~PolygonShape() {
  std::cout << "PolygonShape destructor called!" << std::endl;
}

ShapeType PolygonShape::GetType() const {
  return POLYGON;
}

Shape* PolygonShape::Clone() const {
  return new PolygonShape(localVertices);
}

float PolygonShape::GetMomentOfInertia() const {
  // TODO:
  // We need to compute the moment of inertia of the polygon correctly!!!
  return 5000;
}

Vec2 PolygonShape::EdgeAt(int index) const {
  int currVertex = index;
  int nextVertex = (index + 1) % worldVertices.size();
  return worldVertices[nextVertex] - worldVertices[currVertex];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, Vec2& axis, Vec2& point) const {
  float separation = std::numeric_limits<float>::lowest();
  // Loop all the vertices of "this" polygon
  for (int i = 0; i < this->worldVertices.size(); i++) {
    Vec2 va = this->worldVertices[i];
    Vec2 normal = this->EdgeAt(i).Normal();
    // Loop all the vertices of the "other" polygon
    float minSep = std::numeric_limits<float>::max();
    Vec2 minVertex;
    for (int j = 0; j < other->worldVertices.size(); j++) {
      Vec2 vb = other->worldVertices[j];
      float proj = (vb - va).Dot(normal);
      if (proj < minSep) {
        minSep = proj;
        minVertex = vb;
      }
    }
    if (minSep > separation) {
      separation = minSep;
      axis = this->EdgeAt(i);
      point = minVertex;
    }
  }
  return separation;
}

void PolygonShape::UpdateVertices(float angle, const Vec2& position) {
  // Loop all the vertices, transforming from local to world space
  for (int i = 0; i < localVertices.size(); i++) {
    // First rotate, then we translate
    worldVertices[i] = localVertices[i].Rotate(angle);
    worldVertices[i] += position;
  }
}

BoxShape::BoxShape(float width, float height) {
  this->width = width;
  this->height = height;

  // Load the vertices of the box polygon
  localVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
  localVertices.push_back(Vec2(+width / 2.0, -height / 2.0));
  localVertices.push_back(Vec2(+width / 2.0, +height / 2.0));
  localVertices.push_back(Vec2(-width / 2.0, +height / 2.0));

  worldVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
  worldVertices.push_back(Vec2(+width / 2.0, -height / 2.0));
  worldVertices.push_back(Vec2(+width / 2.0, +height / 2.0));
  worldVertices.push_back(Vec2(-width / 2.0, +height / 2.0));
}

BoxShape::~BoxShape() {
  // TODO: ...
}

ShapeType BoxShape::GetType() const {
  return BOX;
}

Shape* BoxShape::Clone() const {
  return new BoxShape(width, height);
}

float BoxShape::GetMomentOfInertia() const {
  // For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
  // But this still needs to be multiplied by the rigidbody's mass
  return (0.083333) * (width * width + height * height);
}
