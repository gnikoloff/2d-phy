#include <emscripten/bind.h>

#include "World.cpp"
#include "rigidbody/Body.cpp"
#include "rigidbody/Shape.cpp"
#include "collisions/CollisionDetection.cpp"
#include "constraints/Constraint.cpp"
#include "constraints/PenetrationConstraint.cpp"
#include "constraints/JointConstraint.cpp"
#include "math/Vec2.cpp"
#include "math/VecN.cpp"
#include "math/MatMN.cpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(Phy2D) {
  register_vector<Body*>("vector<Body*>");
  register_vector<Vec2>("vector<Vec2>");

  // World
  class_<World>("World")
    .constructor<float, float, float>()
    .property("gravity", &World::GetGravity, &World::SetGravity)
    .function("AddBody", &World::AddBody, allow_raw_pointers())
    .function("GetBodies", &World::GetBodies, allow_raw_pointers())
    .function("GetBody", &World::GetBody, allow_raw_pointers())
    .function("AddJointConstraint", &World::AddJointConstraint, allow_raw_pointers())
    .function("Update", &World::Update)
    ;

  // Body
  class_<Body>("Body")
    .constructor<float, float, float, float>()
    .constructor<float, float, float, float, float>()
    .constructor<const std::vector<Vec2>, float, float, float, float, float>()


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
    .property("boundingCircleRadius", &Body::GetBoundingCircleRadius)

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

  class_<Constraint>("Constraint")
    .constructor<>()
    ;

  class_<JointConstraint>("JointConstraint")
    .constructor<>()
    .constructor<Body*, Body*, Vec2&>()
    .function("GetBody", &JointConstraint::GetBody, allow_raw_pointers())
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