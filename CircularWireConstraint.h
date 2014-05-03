#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void Draw() const;
  void Apply();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
