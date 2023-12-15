#ifndef VEC2_H
#define VEC2_H

struct Vec2 {
  float x;
  float y;

  Vec2();
  Vec2(float x, float y);
  ~Vec2() = default;

  float GetX() const;
  void SetX(const float x);
  float GetY() const;
  void SetY(const float y);

  void Zero();

  void Add(const Vec2& v);
  void Sub(const Vec2& v);
  void Scale(const float n);
  bool Equals(const Vec2& v);
  Vec2 Rotate(const float angle) const;

  float Magnitude() const;
  float MagnitudeSquared() const;

  Vec2& Normalize();
  Vec2 UnitVector() const;
  Vec2 Normal() const;

  float Dot(const Vec2& v) const;
  float Cross(const Vec2& v) const;

  Vec2& operator = (const Vec2& v);
  bool operator == (const Vec2& v) const;
  bool operator != (const Vec2& v) const;

  Vec2 operator + (const Vec2& v) const;
  Vec2 operator - (const Vec2& v) const;
  Vec2 operator * (const float n) const;
  Vec2 operator / (const float n) const;
  Vec2 operator - ();

  Vec2& operator += (const Vec2& v);
  Vec2& operator -= (const Vec2& v);
  Vec2& operator *= (const float n);
  Vec2& operator /= (const float n);
};

#endif
