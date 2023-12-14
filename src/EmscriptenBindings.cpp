#include <emscripten/bind.h>

#include "World.cpp"
#include "Body.cpp"
#include "CollisionDetection.cpp"
#include "Constraint.cpp"
#include "Vec2.cpp"
#include "VecN.cpp"
#include "MatMN.cpp"
#include "Shape.cpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(Phy2D) {
  register_vector<Body*>("vector<Body*>");

  // World
  class_<World>("World")
    .constructor<float>()
    .function("AddBody", &World::AddBody, allow_raw_pointers())
    .function("GetBodies", &World::GetBodies, allow_raw_pointers())
    .function("Update", &World::Update)
  ;

  // Body
  class_<Body>("Body")
    .constructor<float, float, float, float>()
    .constructor<float, float, float, float, float>()

    .property("position", &Body::GetPosition, &Body::SetPosition)
    .property("velocity", &Body::GetVelocity, &Body::SetVelocity)
    .property("acceleration", &Body::GetAcceleration, &Body::SetAcceleration)
    .property("rotation", &Body::GetRotation, &Body::SetRotation)
    .property("angularVelocity", &Body::GetAngularVelocity, &Body::SetAngularVelocity)
    .property("angularAcceleration", &Body::GetAngularAcceleration, &Body::SetAngularAcceleration)
    .property("sumForces", &Body::GetSumForces, &Body::SetSumForces)
    .property("sumTorque", &Body::GetSumTorque, &Body::SetSumTorque)
    .property("mass", &Body::GetMass, &Body::SetMass)
    .property("invMass", &Body::GetInvMass, &Body::SetInvMass)
    .property("I", &Body::GetI, &Body::SetI)
    .property("invI", &Body::GetInvI, &Body::SetInvI)
    .property("restitution", &Body::GetRestitution, &Body::SetRestitution)
    .property("friction", &Body::GetFriction, &Body::SetFriction)


    .function("IsStatic", &Body::IsStatic)
    .function("AddForce", &Body::AddForce)
    .function("AddTorque", &Body::AddTorque)
    .function("ClearForces", &Body::ClearForces)
    .function("ClearTorque", &Body::ClearTorque)
  ;

  // Shape
  enum_<ShapeType>("ShapeType")
    .value("CIRCLE", CIRCLE)
    .value("POLYGON", POLYGON)
    .value("BOX", BOX)
  ;

  class_<CircleShape>("CircleShape")
    .constructor<float>()
    .property("radius", &CircleShape::GetRadius, &CircleShape::SetRadius)
    .function("GetType", &CircleShape::GetType)
  ;

  // Vec2
  class_<Vec2>("Vec2")
    .constructor<>()
    .constructor<float, float>()
    .property("x", &Vec2::GetX, &Vec2::SetX)
    .property("y", &Vec2::GetY, &Vec2::SetY)
    .function("Equals", &Vec2::Equals)
    .function("Add", &Vec2::Add)
    .function("Sub", &Vec2::Sub)
    .function("Scale", &Vec2::Scale)
    .function("Rotate", &Vec2::Rotate)
    .function("Magnitude", &Vec2::Magnitude)
    .function("MagnitudeSquared", &Vec2::MagnitudeSquared)
    .function("Normalize", &Vec2::Normalize)
    .function("UnitVector", &Vec2::UnitVector)
    .function("Normal", &Vec2::Normal)
    .function("Dot", &Vec2::Dot)
    .function("Cross", &Vec2::Cross)
  ;
}