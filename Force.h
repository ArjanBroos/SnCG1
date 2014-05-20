#pragma once

#include "gfx/mat2.h"

// Abstract base class for forces
class Force {
public:
	// Every force will need to implement these functions

	// Applies this force to its associated particles
	virtual void Apply() = 0;
	// Draws the force between the associated particles
	virtual void Draw() const = 0;

	// Returns a jacobian matrix appropriate for this force
	virtual Mat2 GetJacobian(int flags) const = 0;

	int m_ID;	// Unique ID, to be assigned by ParticleSystem
};