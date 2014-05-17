#pragma once

// Abstract base class for forces
class Force {
public:
	// Every force will need to implement these functions

	// Applies this force to its associated particles
	virtual void Apply() = 0;
	// Draws the force between the associated particles
	virtual void Draw() const = 0;

	int m_ID;	// Unique ID, to be assigned by ParticleSystem
};