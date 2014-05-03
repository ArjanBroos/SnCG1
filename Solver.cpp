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