#pragma once

#include "Particle.h"
#include "Constraint.h"

class LineConstraint : public Constraint {
 public:
  LineConstraint(Particle *p1, Vec2f start, Vec2f direction);

  void Draw() const;
  double getC();
  double getCdot();
  std::vector<Vec2f> getJ();
  std::vector<Vec2f> getJdot();
  std::vector<Particle> getParticles();
  gfx::Vec2f normalPerpendicularVector();

 private:

  Particle * const m_p1;
  Vec2f m_start;
  Vec2f m_direction;
};
