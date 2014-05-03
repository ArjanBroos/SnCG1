#pragma once

#include "Particle.h"
#include <vector>
#include "Force.h"
#include "Constraint.h"

// A system used for the simulation of particles
class ParticleSystem {
public:
	// Frees any allocated memory
	~ParticleSystem();

	// Adds a particle to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	void							AddParticle(Particle* particle);
	// Adds a force to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	void							AddForce(Force* force);
	// Adds a constraint to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	void							AddConstraint(Constraint* constraint);

	// Get the particles in this system
	// Non-const reference is returned, so it allows our particles to be modified through the returned vector
	std::vector<Particle*>&			GetParticles();
	// Get the forces working in this system
	const std::vector<Force*>&		GetForces() const;
	// Get the constraints on this system
	const std::vector<Constraint*>&	GetConstraints() const;

	// Derivative evaluation
	void							DerivEval(std::vector<Vec2f>& derivatives);

	// Resets the system back to its initial state
	void							Reset();
	// Removes everything from the system
	void							Clear();

private:
	std::vector<Particle*>			particles;
	std::vector<Force*>				forces;
	std::vector<Constraint*>		constraints;
};