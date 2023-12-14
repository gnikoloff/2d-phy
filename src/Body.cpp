#include "Body.h"
#include <math.h>
#include <iostream>

Body::Body(float radius, float x, float y, float mass): Body(CircleShape(radius), x, y, mass) {

}

Body::Body(float width, float height, float x, float y, float mass): Body(BoxShape(width, height), x, y, mass) {
  
}

Body::Body(const Shape& shape, float x, float y, float mass) {
    this->shape = shape.Clone();
    this->position = Vec2(x, y);
    this->velocity = Vec2(0, 0);
    this->acceleration = Vec2(0, 0);
    this->rotation = 0.0;
    this->angularVelocity = 0.0;
    this->angularAcceleration = 0.0;
    this->sumForces = Vec2(0, 0);
    this->sumTorque = 0.0;
    this->restitution = 0.6;
    this->friction = 0.7;
    this->mass = mass;
    if (mass != 0.0) {
        this->invMass = 1.0 / mass;
    } else {
        this->invMass = 0.0;
    }
    I = shape.GetMomentOfInertia() * mass;
    if (I != 0.0) {
        this->invI = 1.0 / I;
    } else {
        this->invI = 0.0;
    }
  //     std::cout << x << std::endl;
  // std::cout << y << std::endl;
  std::cout << this->mass << std::endl;
    this->shape->UpdateVertices(rotation, position);
    std::cout << "Body constructor called!" << std::endl;
}

Body::~Body() {
    delete shape;
    std::cout << "Body destructor called!" << std::endl;
}

Vec2 Body::GetPosition() const {
  return position;
}

void Body::SetPosition(const Vec2& v) {
  this->position = v;
}

Vec2 Body::GetVelocity() const {
  return velocity;
}

void Body::SetVelocity(const Vec2& v) {
  this->velocity = v;
}

Vec2 Body::GetAcceleration() const {
  return acceleration;
}

void Body::SetAcceleration(const Vec2& v) {
  this->acceleration = v;
}

float Body::GetRotation() const {
  return rotation;
}

void Body::SetRotation(const float rotation) {
  this->rotation = rotation;
}

float Body::GetAngularVelocity() const {
  return angularVelocity;
}

void Body::SetAngularVelocity(const float angularVelocity) {
  this->angularVelocity = angularVelocity;
}

float Body::GetAngularAcceleration() const {
  return angularAcceleration;
}

void Body::SetAngularAcceleration(const float angularAcceleration) {
  this->angularAcceleration = angularAcceleration;
}

Vec2 Body::GetSumForces() const {
  return sumForces;
}

void Body::SetSumForces(const Vec2& sumForces) {
  this->sumForces = sumForces;
}

float Body::GetSumTorque() const {
  return sumTorque;
}

void Body::SetSumTorque(const float sumTorque) {
  this->sumTorque = sumTorque;
}

float Body::GetMass() const {
  return mass;
}

void Body::SetMass(const float mass) {
  this->mass = mass;
}

float Body::GetInvMass() const {
  return invMass;
}

void Body::SetInvMass(const float invMass) {
  this->invMass = invMass;
}

float Body::GetI() const {
  return I;
}

void Body::SetI(const float I) {
  this->I = I;
}

float Body::GetInvI() const {
  return invI;
}

void Body::SetInvI(const float I) {
  this->I = I;
}

float Body::GetRestitution() const {
  return restitution;
}

void Body::SetRestitution(const float restitution) {
  this->restitution = restitution;
}

float Body::GetFriction() const {
  return friction;
}

void Body::SetFriction(const float friction) {
  this->friction = friction;
}

bool Body::IsStatic() const {
    const float epsilon = 0.005f;
    return fabs(invMass - 0.0) < epsilon;
}

void Body::AddForce(const Vec2& force) {
    sumForces += force;
}

void Body::AddTorque(float torque) {
    sumTorque += torque;
}

void Body::ClearForces() {
    sumForces = Vec2(0.0, 0.0);
}

void Body::ClearTorque() {
    sumTorque = 0.0;
}

Vec2 Body::LocalSpaceToWorldSpace(const Vec2& point) const {
    Vec2 rotated = point.Rotate(rotation);
	return rotated + position;
}

Vec2 Body::WorldSpaceToLocalSpace(const Vec2& point) const {
    float translatedX = point.x - position.x;
    float translatedY = point.y - position.y;
    float rotatedX = cos(-rotation) * translatedX - sin(-rotation) * translatedY;
    float rotatedY = cos(-rotation) * translatedY + sin(-rotation) * translatedX;
	return Vec2(rotatedX, rotatedY);
}

void Body::ApplyImpulseLinear(const Vec2& j) {
    if (IsStatic())
        return;
    velocity += j * invMass;
}

void Body::ApplyImpulseAngular(const float j) {
    if (IsStatic())
        return;
    angularVelocity += j * invI;
}

void Body::ApplyImpulseAtPoint(const Vec2& j, const Vec2& r) {
    if (IsStatic())
        return;
    velocity += j * invMass;
    angularVelocity += r.Cross(j) * invI;
}

void Body::IntegrateForces(const float dt) {
    if (IsStatic())
        return;

    // Find the acceleration based on the forces that are being applied and the mass
    acceleration = sumForces * invMass;

    // Integrate the acceleration to find the new velocity
    velocity += acceleration * dt;

    // Find the angular acceleration based on the torque that is being applied and the moment of inertia
    angularAcceleration = sumTorque * invI;

    // Integrate the angular acceleration to find the new angular velocity
    angularVelocity += angularAcceleration * dt;

    // Clear all the forces and torque acting on the object before the next physics step
    ClearForces();
    ClearTorque();
}

void Body::IntegrateVelocities(const float dt) {
    if (IsStatic())
        return;

    // Integrate the velocity to find the new position
    position += velocity * dt;

    // Integrate the angular velocity to find the new rotation angle
    rotation += angularVelocity * dt;

    // Update the vertices to adjust them to the new position/rotation
    shape->UpdateVertices(rotation, position);
}
