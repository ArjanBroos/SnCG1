#include "ParticleSystem.h"
#include "gfx/vec2.h"

// Frees any allocated memory
ParticleSystem::~ParticleSystem() {
	Clear();
}

// Adds a particle to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
void ParticleSystem::AddParticle(Particle* particle) {
	particles.push_back(particle);
}

// Adds a force to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
void ParticleSystem::AddForce(Force* force) {
	forces.push_back(force);
}

// Adds a constraint to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
void ParticleSystem::AddConstraint(Constraint* constraint) {
	constraints.push_back(constraint);
}

// Get the particles in this system
std::vector<Particle*>& ParticleSystem::GetParticles() {
	return particles;
}

// Get the forces working in this system
const std::vector<Force*>& ParticleSystem::GetForces() const {
	return forces;
}

// Get the constraints on this system
const std::vector<Constraint*>& ParticleSystem::GetConstraints() const {
	return constraints;
}

// Derivative evaluation
void ParticleSystem::DerivEval(std::vector<Vec2f>& derivatives) {
	// Clear force accumulators
	for (auto p = particles.begin(); p != particles.end(); p++)
		(*p)->m_ForceAcc = Vec2f(0.f, 0.f);

	// Apply all forces
	for (auto f = forces.begin(); f != forces.end(); f++)
		(*f)->Apply();

	// Calculate derivatives
	for (auto pi = particles.begin(); pi != particles.end(); pi++) {
		Particle* p = *pi;
		derivatives.push_back(p->m_Velocity);				// x' = v
		derivatives.push_back(p->m_ForceAcc / p->m_Mass);	// v' = f / m
	}
}

// Resets the system back to its initial state
void ParticleSystem::Reset() {
	for (auto p = particles.begin(); p != particles.end(); p++)
		(*p)->Reset();
}

// Removes everything from the system
void ParticleSystem::Clear() {
	for (auto p = particles.begin(); p != particles.end(); p++)
		delete *p;
	particles.clear();
	for (auto f = forces.begin(); f != forces.end(); f++)
		delete *f;
	forces.clear();
	for (auto c = constraints.begin(); c != constraints.end(); c++)
		delete *c;
	constraints.clear();
}