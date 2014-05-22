#pragma once

#include "Particle.h"
#include <vector>
#include "Force.h"
#include "Constraint.h"
#include "CollidableLineSegment.h"

// A system used for the simulation of particles
class ParticleSystem {
public:
	// Sets empty state
	ParticleSystem();
	// Frees any allocated memory
	~ParticleSystem();

	// Adds a particle to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	// Returns the ID of the added particle
	int								AddParticle(Particle* particle);
	// Adds a force to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	// Returns the ID of the added force
	int								AddForce(Force* force);
	// Adds a constraint to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	// Returns the id of the added constraint
	int								AddConstraint(Constraint* constraint);

	// Adds a constraint to the system
	// This system will take ownership of the pointer and is therefore responsible for its destruction
	// Returns the id of the added constraint
	int								AddCollidableLineSegment(CollidableLineSegment* cls);

	// Functions for removing parts of the system
	void							RemoveParticle(int particleID);
	void							RemoveForce(int forceID);
	void							RemoveConstraint(int constraintID);

	// Get the particles in this system
	// Non-const reference is returned, so it allows our particles to be modified through the returned vector
	std::vector<Particle*>&			GetParticles();
	// Get the forces working in this system
	const std::vector<Force*>&		GetForces() const;
	// Get the constraints on this system
	const std::vector<Constraint*>&	GetConstraints() const;
	// Get the particle closest to position, excluding particle with particleID
	Particle*						GetClosestParticle(Vec2f position, int particleID);
	// Get the CollidableLineSegments in this system
	const std::vector<CollidableLineSegment*>&	GetCollidableLineSegment() const;

	// Derivative evaluation
	void							DerivEval(std::vector<Vec2f>& derivatives);

	// Resets the system back to its initial state
	void							Reset();
	// Removes everything from the system
	void							Clear();

	// Compute and apply the constraint forces
	void							ComputeApplyConstForce();

private:
	// Components of the particle system
	std::vector<Particle*>			particles;
	std::vector<Force*>				forces;
	std::vector<Constraint*>		constraints;
	std::vector<CollidableLineSegment*> collidableLineSegments;

	// Counters used for assigning unique ID's
	int								particleIDCounter;
	int								forceIDCounter;
	int								constraintIDCounter;
	int								clsCounter;

	// Not kept locally in functions, to prevent constant reallocation
	vector<vector<float>>			J;
	vector<vector<float>>			W;
	vector<vector<float>>			Jt;
	vector<vector<float>>			Jdot;
	vector<float>					qdot;
	vector<float>					Q;
	vector<float>					C;
	vector<float>					Cdot;
	vector<vector<float>>			JWJt;
	vector<vector<float>>			JW;
	vector<float>					rightHandSide;
	vector<float>					fhat;
	vector<float>					Qhat;
};