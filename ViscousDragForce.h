#pragma once

#include "Force.h"
#include "Particle.h"

// Applies viscous drag (friction)
class ViscousDragForce : public Force {
public:
	ViscousDragForce(Particle* p, float drag);

	// Applies this force to its associated particle
	void		Apply();
	// Draws the force on the associated particle
	void		Draw() const;
	// Returns a jacobian matrix for this force wrt velocity
	Mat2		GetJacobian(int flags) const;
	// Returns the index of the particle
	unsigned	GetPIndex() const;

private:
	Particle*	particle;	// Particle to apply force to
	float		drag;		// Drag coefficient
};