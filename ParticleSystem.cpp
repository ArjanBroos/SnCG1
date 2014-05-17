#include "ParticleSystem.h"
#include "gfx/vec2.h"
#include "linearSolver.h"
#include "util.h"

using namespace std;

// Sets empty state
ParticleSystem::ParticleSystem() {
	particleIDCounter = 0;
	forceIDCounter = 0;
	constraintIDCounter = 0;
}

// Frees any allocated memory
ParticleSystem::~ParticleSystem() {
	Clear();
}

// Adds a particle to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
int ParticleSystem::AddParticle(Particle* particle) {
	particles.push_back(particle);
	particle->m_ID = particleIDCounter++;
	particle->m_Number = particles.size() -  1;
	return particle->m_ID;
}

// Adds a force to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
int ParticleSystem::AddForce(Force* force) {
	forces.push_back(force);
	force->m_ID = forceIDCounter++;
	return force->m_ID;
}

// Adds a constraint to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
int ParticleSystem::AddConstraint(Constraint* constraint) {
	constraints.push_back(constraint);
	constraint->m_ID = constraintIDCounter++;
	return constraint->m_ID;
}

// Removes a particle from the system
// ! Assumes particles are in ordering of ID !
void ParticleSystem::RemoveParticle(int particleID) {
	int index = BinarySearch<Particle*>(particles, particleID);
	if (index != -1) {
		delete particles[index];
		particles.erase(particles.begin() + index);
	}
}

// Removes a force from the system
// ! Assumes forces are in ordering of ID !
void ParticleSystem::RemoveForce(int forceID) {
	int index = BinarySearch<Force*>(forces, forceID);
	if (index != -1) {
		delete forces[index];
		forces.erase(forces.begin() + index);
	}
}

// Removes a constraint from the system
// ! Assumes constraints are in ordering of ID !
void ParticleSystem::RemoveConstraint(int constraintID) {
	int index = BinarySearch<Constraint*>(constraints, constraintID);
	if (index != -1) {
		delete constraints[index];
		constraints.erase(constraints.begin() + index);
	}
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

// Return the particle closest to (x, y), excluding particle with particleID
Particle* ParticleSystem::GetClosestParticle(Vec2f position, int particleID) {
	Particle* closest = nullptr;
	float mind = 1e30f;	// Minimum squared distance so far
	for (auto p = particles.begin(); p != particles.end(); p++) {
		if ((*p)->m_ID == particleID) continue;
		float ds = norm2(position - (*p)->m_Position); // Distance squared
		if (ds < mind) {
			closest = *p;
			mind = ds;
		}
	}
	return closest;
}

// Derivative evaluation
void ParticleSystem::DerivEval(std::vector<Vec2f>& derivatives) {
	// Clear force accumulators
	for (auto p = particles.begin(); p != particles.end(); p++)
		(*p)->m_ForceAcc = Vec2f(0.f, 0.f);

	// Apply all forces
	for (auto f = forces.begin(); f != forces.end(); f++)
		(*f)->Apply();

	ComputeApplyConstForce();

	// Calculate derivatives
	for (auto pi = particles.begin(); pi != particles.end(); pi++) {		
		Particle* p = *pi;
		derivatives.push_back(p->m_Velocity);				// x' = v
		derivatives.push_back(p->m_ForceAcc / p->m_Mass);	// v' = f / m
	}
}

void ParticleSystem::ComputeApplyConstForce(){

	if (constraints.size()==0 || particles.size()==0){
		return;
	}

	int n = 2;
	float ks = 0.31f;
	float kd = 0.62f;
	//Make J
	J.resize(constraints.size());
	for (unsigned i = 0; i < constraints.size(); i++) {
		J[i].resize(particles.size() * n);
		for (unsigned j = 0; j < particles.size() * n; j++) {
			J[i][j] = 0;
		}
	}

	for (unsigned i = 0; i < constraints.size(); i++) {
		vector<Vec2f> cons = constraints[i]->getJ();
		vector<Particle> part = constraints[i]->getParticles();
		for (unsigned j = 0; j < part.size(); j++) {
			for (int o = 0; o < n; o++) {
				J[i][part[j].m_Number*n + o] = cons[j][o];
			}
		}
	}
	

	//Make W
	W.resize(particles.size() * n);
	for (unsigned i = 0; i < particles.size() *n; i++) {
		W[i].resize(particles.size() * n);
		for (unsigned j = 0; j < particles.size()*n; j++) {
			W[i][j] = 0;
		}
	}

	for (unsigned i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			W[n*i + o][n*i + o] = 1/particles[i]->m_Mass;
		}
	}

	//Make Jt
	Jt.resize(particles.size() * n);
	for (unsigned i = 0; i<particles.size() * n; i++) {
		Jt[i].resize(constraints.size());
		for (unsigned j = 0; j < constraints.size(); j++){
			Jt[i][j] = J[j][i];
		}
	}

	//Make Jdot
	Jdot.resize(constraints.size());
	for (unsigned i = 0; i < constraints.size(); i++) {
		Jdot[i].resize(particles.size() * n);
		for (unsigned j = 0; j < particles.size() * n; j++) {
			Jdot[i][j] = 0;
		}
	}

	for (unsigned i = 0; i < constraints.size(); i++) {
		vector<Vec2f> cons = constraints[i]->getJdot();
		vector<Particle> part = constraints[i]->getParticles();
		for (unsigned j = 0; j < part.size(); j++) {
			for (int o = 0; o < n; o++) {
				Jdot[i][part[j].m_Number*n + o] = cons[j][o];
			}
		}
	}

	//Make qdot
	qdot.resize(particles.size()*n);
	for (unsigned i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			qdot[n*i + o] = particles[i]->m_Velocity[o];
		}
	}

	//Make Q
	Q.resize(particles.size()*n);
	for (unsigned i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			Q[n*i + o] = particles[i]->m_ForceAcc[o];
		}
	}

	//Make C
	C.resize(constraints.size());
	for (unsigned i = 0; i<constraints.size(); i++) {
		C[i] = (float)constraints[i]->getC();
	}

	//Make Cdot
	Cdot.resize(constraints.size());
	for (unsigned i = 0; i<constraints.size(); i++) {
		Cdot[i] = (float)constraints[i]->getCdot();
	}

	//Make JWJt and also JW
	JW = mul(J, W);
	JWJt = mul(JW, Jt);

	//Make −Jdot*qdot − JWQ with feedback
	rightHandSide = diffEqual(diffEqual(diffEqual(
		timesScalar(vecmul(Jdot, qdot), -1), vecmul(JW,Q))
		,timesScalar(C,ks))
		, timesScalar(Cdot, kd));


	//Solve
	implicitMatrix *M = new implicitMatrix(JWJt);
	double* lambda = new double[constraints.size()];
	double* r = new double[constraints.size()];

	for (unsigned i = 0; i < rightHandSide.size(); i++){
		r[i] = rightHandSide[i];
	}

	int d = 100;
	ConjGrad(constraints.size(), M, lambda, r, 0.0000000000001, &d);
	delete M;
	delete[] r;

	//Make Qhat
	fhat.resize(constraints.size());
	for (unsigned i = 0; i < constraints.size(); i++){
		fhat[i] = (float)lambda[i];
	}
	delete[] lambda;

	Qhat = vecmul(Jt, fhat);
	
	//Assign forces
	for (unsigned int i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			particles[i]->m_ForceAcc[o] += Qhat[2 * i + o];
		}
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