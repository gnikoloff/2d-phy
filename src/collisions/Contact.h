#ifndef CONTACT_H
#define CONTACT_H

#include "../math/Vec2.h"
#include "../rigidbody/Body.h"

struct Contact {
  Body* a;
  Body* b;

  Vec2 start;
  Vec2 end;

  Vec2 normal;
  float depth;
};

#endif
