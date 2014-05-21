#pragma once

#include "Particle.h"
#include "Force.h"


class SpringAngleForce : public Force {
 public:
	 SpringAngleForce(Particle *p1,Particle *joint, Particle * p2, double angle, double ks, double kd);

  // Draws the force between the associated particles
  void		Draw() const;
  // Applies this force to its associated particles
  void		Apply();
  // Returns a jacobian matrix for this force
  // Use the flags parameter to specify which particle
  // Also use the flags parameter to specify with respect to what we calculate this matrix (position or velocity)
  // For example: GetJacobian(JF_P1 | JF_POS) will return the jacobian matrix for particle #1, wrt position
  Mat2		GetJacobian(int flags) const;

 private:

  Particle* m_p1;   // particle 1
  Particle* m_p2;   // particle 2 
  Particle* m_joint;   // particle j
  double	m_angle;     // rest length
  double	m_ks, m_kd; // spring strength constants
};
