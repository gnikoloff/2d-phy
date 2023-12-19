# 2d-phy

Lightweight 2D game physics engine.

## Platforms

- Written in C++
- Currently compiling to Emscripten only
  - Uses Embind to expose C++ classes and value types to Javascript
  - Generates Typescript definitions from Embind declarations

## Features

- Rigid body with motion types
  - Dynamic
  - Static
  - Kinematic
- Contacts with friction and restitution
- Collision shapes
  - Circle
  - Box
  - Convex polygon
- Broadphase and narrowphase collision detection
- Constraints
  - Joint
  - Penetration
- Constraints solver
- Forces

  - Drag
  - Friction
  - Spring
  - Gravitation

  ## References and readings

- [Game Physics Engine Programming](https://pikuma.com/courses/game-physics-engine-programming)
- [Game Physics Engine Development](https://www.amazon.de/-/en/Ian-Millington/dp/0123819768)
