export interface vector<Body*> {
  push_back(_0: Body): void;
  resize(_0: number, _1: Body): void;
  size(): number;
  set(_0: number, _1: Body): boolean;
  get(_0: number): any;
  delete(): void;
}

export interface vector<Vec2> {
  push_back(_0: Vec2): void;
  resize(_0: number, _1: Vec2): void;
  size(): number;
  set(_0: number, _1: Vec2): boolean;
  get(_0: number): any;
  delete(): void;
}

export interface World {
  GetBodies(): vector<Body*>;
  AddBody(_0: Body): void;
  Update(_0: number): void;
  delete(): void;
}

export interface Body {
  position: Vec2;
  velocity: Vec2;
  acceleration: Vec2;
  sumForces: Vec2;
  rotation: number;
  angularVelocity: number;
  angularAcceleration: number;
  sumTorque: number;
  mass: number;
  invMass: number;
  I: number;
  invI: number;
  restitution: number;
  friction: number;
  readonly boundingCircleRadius: number;
  AddForce(_0: Vec2): void;
  ClearForces(): void;
  ClearTorque(): void;
  IsStatic(): boolean;
  AddTorque(_0: number): void;
  delete(): void;
}

export interface ShapeTypeValue<T extends number> {
  value: T;
}
export type ShapeType = ShapeTypeValue<0>|ShapeTypeValue<1>|ShapeTypeValue<2>;

export interface CircleShape {
  radius: number;
  GetType(): ShapeType;
  delete(): void;
}

export interface Vec2 {
  x: number;
  y: number;
  Normalize(): Vec2;
  UnitVector(): Vec2;
  Normal(): Vec2;
  Add(_0: Vec2): void;
  Sub(_0: Vec2): void;
  Equals(_0: Vec2): boolean;
  Scale(_0: number): void;
  Rotate(_0: number): Vec2;
  Magnitude(): number;
  MagnitudeSquared(): number;
  Dot(_0: Vec2): number;
  Cross(_0: Vec2): number;
  delete(): void;
}

export interface MainModule {
  vector<Body*>: {new(): vector<Body*>};
  vector<Vec2>: {new(): vector<Vec2>};
  World: {new(_0: number): World};
  Body: {new(_0: number, _1: number, _2: number, _3: number): Body; new(_0: number, _1: number, _2: number, _3: number, _4: number): Body; new(_0: vector<Vec2>, _1: number, _2: number, _3: number, _4: number, _5: number): Body};
  ShapeType: {CIRCLE: ShapeTypeValue<0>, POLYGON: ShapeTypeValue<1>, BOX: ShapeTypeValue<2>};
  CircleShape: {new(_0: number): CircleShape};
  Vec2: {new(): Vec2; new(_0: number, _1: number): Vec2};
}
