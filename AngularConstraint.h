#pragma once

#include "Particle.h"
#include "Constraint.h"

class AngularConstraint : public Constraint {
 public:
	 AngularConstraint(Particle *p1, Particle * p_joint, Particle * p2, double angle);

  void Draw() const;
  double getC();
  double getCdot();
  std::vector<Vec2f> getJ();
  std::vector<Vec2f> getJdot();
  std::vector<Particle> getParticles();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  Particle * const m_joint;
  double const m_angle;
};
