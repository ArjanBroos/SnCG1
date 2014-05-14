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

void RungeKutta4Step(ParticleSystem& particleSystem, float dt) {
	auto& particles = particleSystem.GetParticles();

	// Save current state
	std::vector<Vec2f> positions;
	std::vector<Vec2f> velocities;
	for (auto i = particles.begin(); i != particles.end(); i++) {
		positions.push_back((*i)->m_Position);
		velocities.push_back((*i)->m_Velocity);
	}

	// Evaluate derivatives
	std::vector<Vec2f> derivatives;
	particleSystem.DerivEval(derivatives);

	// Calculate k1
	std::vector<Vec2f> k1_pos;
	std::vector<Vec2f> k1_vel;
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		Particle* p = particles[pi];
		const unsigned di = pi*2;
		k1_pos.push_back(dt * derivatives[di]);
		k1_vel.push_back(dt * derivatives[di+1]);
	}

	// Calculate k2
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		particles[pi]->m_Position += 0.5f * k1_pos[pi];
		particles[pi]->m_Velocity += 0.5f * k1_vel[pi];
	}
	derivatives.clear();
	particleSystem.DerivEval(derivatives);
	std::vector<Vec2f> k2_pos;
	std::vector<Vec2f> k2_vel;
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		const unsigned di = pi*2;
		k2_pos.push_back(dt * derivatives[di]);
		k2_vel.push_back(dt * derivatives[di+1]);
	}

	// Calculate k3
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		particles[pi]->m_Position = positions[pi] + 0.5f * k2_pos[pi];
		particles[pi]->m_Velocity = velocities[pi] + 0.5f * k2_vel[pi];
	}
	derivatives.clear();
	particleSystem.DerivEval(derivatives);
	std::vector<Vec2f> k3_pos;
	std::vector<Vec2f> k3_vel;
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		const unsigned di = pi*2;
		k3_pos.push_back(dt * derivatives[di]);
		k3_vel.push_back(dt * derivatives[di+1]);
	}

	// Calculate k4
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		particles[pi]->m_Position = positions[pi] + k3_pos[pi];
		particles[pi]->m_Velocity = velocities[pi] + k3_vel[pi];
	}
	derivatives.clear();
	particleSystem.DerivEval(derivatives);
	std::vector<Vec2f> k4_pos;
	std::vector<Vec2f> k4_vel;
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		const unsigned di = pi*2;
		k4_pos.push_back(dt * derivatives[di]);
		k4_vel.push_back(dt * derivatives[di+1]);
	}

	// Update state
	for (unsigned pi = 0; pi < particles.size(); pi++) {
		Particle* p = particles[pi];
		p->m_Position = positions[pi] + 
			(k1_pos[pi] + 2.f * k2_pos[pi] + 2.f * k3_pos[pi] + k4_pos[pi]) / 6.f;
		p->m_Velocity = velocities[pi] +
			(k1_vel[pi] + 2.f * k2_vel[pi] + 2.f * k3_vel[pi] + k4_vel[pi]) / 6.f;
	}
}