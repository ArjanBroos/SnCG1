#include "Particle.h"
#include "ParticleSystem.h"

#include <vector>

void ExplicitEulerStep(ParticleSystem& particleSystem, float dt) {
	// Retrieve current state
	auto& particles = particleSystem.GetParticles();

	// Evaluate derivatives
	std::vector<Vec2f> derivatives;
	particleSystem.DerivEval(derivatives);

	// Set new state
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		Particle* p = particles[pi];
		const unsigned di = pi*2; // Derivative Index
		p->m_Position += derivatives[di] * dt;
		p->m_Velocity += derivatives[di+1] * dt;
	}
}

void MidPointStep(ParticleSystem& particleSystem, float dt) {
	auto& particles = particleSystem.GetParticles();

	// Save current state
	std::vector<Vec2f> positions;
	std::vector<Vec2f> velocities;
	for (auto i = particles.begin(); i != particles.end(); i++) {
		positions.push_back((*i)->m_Position);
		velocities.push_back((*i)->m_Velocity);
	}

	// Evaluate first derivatives
	std::vector<Vec2f> derivatives;
	particleSystem.DerivEval(derivatives);

	// Calculate mid-points
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		Particle* p = particles[pi];
		const unsigned di = pi*2;
		p->m_Position += derivatives[di] * dt * 0.5f;
		p->m_Velocity += derivatives[di+1] * dt * 0.5f;
	}

	// Evaluate second derivatives
	derivatives.clear();
	particleSystem.DerivEval(derivatives);

	// Set new state
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		Particle* p = particles[pi];
		const unsigned di = pi*2;
		p->m_Position = positions[pi] + derivatives[di] * dt;
		p->m_Velocity = velocities[pi] + derivatives[di+1] * dt;
	}
}