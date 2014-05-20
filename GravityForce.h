#pragma once

#include "Force.h"
#include "Particle.h"

// A force that models gravity
class GravityForce : public Force {
public:
	GravityForce(Particle* particle);

	// Applies this force to its associated particle
	void		Apply();
	// Draws the force on the associated particle
	void		Draw() const;
	// Returns a jacobian matrix for this force (all 0)
	Mat2		GetJacobian(int flags) const;

private:
	Particle*	particle;
};