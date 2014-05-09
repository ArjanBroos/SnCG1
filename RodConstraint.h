#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);

  void Draw() const;
  double getC();
  double getCdot();
  std::vector<Vec2f> getJ();
  std::vector<Vec2f> getJdot();
  std::vector<Particle> getParticles();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
