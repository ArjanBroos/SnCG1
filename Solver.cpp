#include "Particle.h"
#include "ParticleSystem.h"
#include "SpringForce.h"
#include "GravityForce.h"
#include "ViscousDragForce.h"
#include "linearSolver.h"

#include <vector>

void ExplicitEulerStep(ParticleSystem& particleSystem, float dt) {
	// Retrieve current state
	auto& particles = particleSystem.GetParticles();
	auto& collidableLineSegments = particleSystem.GetCollidableLineSegment();

	for each (CollidableLineSegment* cls in collidableLineSegments){
		cls->handleCollisions(particles);
	}

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

void AddMat2ToMatrix(std::vector<std::vector<float>>& M, const Mat2& m2, unsigned i1, unsigned i2) {
	M[i1][i2]		+= (float)m2[0][0];
	M[i1][i2+1]		+= (float)m2[0][1];
	M[i1+1][i2]		+= (float)m2[1][0];
	M[i1+1][i2+1]	+= (float)m2[1][1];
}

void ImplicitEulerstep(ParticleSystem& particleSystem, float dt) {
	particleSystem.ComputeApplyConstForce();

	std::vector<Particle*> particles = particleSystem.GetParticles();
	std::vector<Force*> forces = particleSystem.GetForces();
	const unsigned N = 2;						// 2D particles
	const unsigned dim = N * particles.size();	// Dimension of particle vector

	// Create mass matrix
	std::vector<std::vector<float>> M(dim, std::vector<float>(dim));
	for (unsigned i = 0; i < dim; i++) {
		for (unsigned j = 0; j < dim; j++) {
			if (i == j) M[i][j] = particles[i/N]->m_Mass;
			else M[i][j] = 0.f;
		}
	}

	// For every force, insert Jacobian into M and result
	std::vector<double> result(dim, 0.0);
	for (unsigned i = 0; i < forces.size(); i++) {
		if (typeid(*forces[i]) == typeid(SpringForce)) {
			const SpringForce* sf = (SpringForce*)forces[i];
			const Mat2 jacPos = sf->GetJacobian(JF_P1 | JF_POS);
			const Mat2 jacVel = sf->GetJacobian(JF_P2 | JF_VEL);
			const Mat2 A = dt*dt * jacPos + dt * jacVel;
			const unsigned p1i = sf->GetP1Index();
			const unsigned p2i = sf->GetP2Index();
			// Insert Jacobians into M
			AddMat2ToMatrix(M, -A, p1i, p2i);
			AddMat2ToMatrix(M, A, p2i, p1i);
			// Insert result vector for this force into result
			const double dts = (double)dt*dt;
			TVec2<double> r1 = (jacPos * particles[p1i]->m_Velocity) * dts;
			TVec2<double> r2 = (jacPos * particles[p2i]->m_Velocity) * dts;
			result[p1i] = (double)r1[0]; result[p1i+1] = (double)r1[1];
			result[p2i] = (double)r2[0]; result[p2i+1] = (double)r2[1];
		}
		if (typeid(*forces[i]) == typeid(ViscousDragForce)) {
			ViscousDragForce* vdf = (ViscousDragForce*)forces[i];
			Mat2 A = dt * vdf->GetJacobian(0);	// Jacobian wrt position is 0, so only wrt velocity
			AddMat2ToMatrix(M, -A, vdf->GetPIndex(), vdf->GetPIndex());
		}
		// Both the Jacobian wrt position and the one wrt to velocity are 0 for gravity, so ignore it
	}

	// Add current values to result
	for (unsigned i = 0; i < dim; i += N) {
		result[i]	+= dt * particles[i/N]->m_ForceAcc[0];
		result[i+1] += dt * particles[i/N]->m_ForceAcc[1];
	}

	// Use conjugate gradient to solve for Delta v
	std::vector<double> delta_v(dim);
	implicitMatrix iM(&M);
	ConjGrad(dim, &iM, &delta_v[0], &result[0], 0.001f, 0);
	
	// Calculate Delta x
	std::vector<double> delta_x(dim);
	for (unsigned i = 0; i < dim; i += N) {
		delta_x[i]		= (particles[i/N]->m_Velocity[0] + delta_v[i]) * dt;
		delta_x[i+1]	= (particles[i/N]->m_Velocity[1] + delta_v[i+1]) * dt;
	}

	// Update positions and velocities
	for (unsigned i = 0; i < particles.size(); i++) {
		Particle* p = particles[i];
		p->m_Position += Vec2f((float)delta_x[i*N], (float)delta_x[i*N+1]);
		p->m_Velocity += Vec2f((float)delta_v[i*N], (float)delta_v[i*N+1]);
		p->m_ForceAcc = Vec2f(0.f, 0.f);
	}

	// Calculate new forces
	for (unsigned i = 0; i < forces.size(); i++) {
		forces[i]->Apply();
	}
}

void MidPointStep(ParticleSystem& particleSystem, float dt) {
	auto& particles = particleSystem.GetParticles();

	auto& collidableLineSegments = particleSystem.GetCollidableLineSegment();

	for each (CollidableLineSegment* cls in collidableLineSegments){
		cls->handleCollisions(particles);
	}

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

	auto& collidableLineSegments = particleSystem.GetCollidableLineSegment();

	for each (CollidableLineSegment* cls in collidableLineSegments){
		cls->handleCollisions(particles);
	}

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