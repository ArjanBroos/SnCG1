#pragma once

#include "Force.h"
#include "Particle.h"

// Applies viscous drag (friction)
class ViscousDragForce : public Force {
public:
	ViscousDragForce(Particle* p, float drag);

	// Applies this force to its associated particles
	void Apply();
	// Draws the force between the associated particles
	void Draw() const;

private:
	Particle*	particle;	// Particle to apply force to
	float		drag;		// Drag coefficient
};