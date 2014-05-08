#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void Draw() const;
  double getC();
  double getCdot();
  std::vector<Vec2f> getJ();
  std::vector<Vec2f> getJdot();
  std::vector<Particle> getParticles();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
