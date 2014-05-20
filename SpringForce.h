#pragma once

#include "Particle.h"
#include "Force.h"

const int JF_P1 = 1;
const int JF_P2 = 2;
const int JF_POS = 4;
const int JF_VEL = 8;

class SpringForce : public Force {
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);

  // Draws the force between the associated particles
  void		Draw() const;
  // Applies this force to its associated particles
  void		Apply();
  // Returns a jacobian matrix for this force
  // Use the flags parameter to specify which particle
  // Also use the flags parameter to specify with respect to what we calculate this matrix (position or velocity)
  // For example: GetJacobian(JF_P1 | JF_POS) will return the jacobian matrix for particle #1, wrt position
  Mat2		GetJacobian(int flags) const;
  // Return the index of particle 1
  unsigned	GetP1Index() const;
  // Return the index of particle 2
  unsigned	GetP2Index() const;

 private:

  Particle* m_p1;   // particle 1
  Particle* m_p2;   // particle 2 
  double	m_dist;     // rest length
  double	m_ks, m_kd; // spring strength constants
};
