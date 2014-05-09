#include "ParticleSystem.h"
#include "gfx/vec2.h"
#include "linearSolver.h"

using namespace std;

// Frees any allocated memory
ParticleSystem::~ParticleSystem() {
	Clear();
}

// Adds a particle to the system
// This system will take ownership of the pointer and is therefore responsible for its destruction
void ParticleSystem::AddParticle(Particle* particle) {
	particles.push_back(particle);
	particle->m_Number = particles.size() - 1;
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

	ComputeApplyConstForce();

	// Calculate derivatives
	for (auto pi = particles.begin(); pi != particles.end(); pi++) {		
		Particle* p = *pi;
		derivatives.push_back(p->m_Velocity);				// x' = v
		derivatives.push_back(p->m_ForceAcc / p->m_Mass);	// v' = f / m
	}
}

void ParticleSystem::ComputeApplyConstForce(){
	int n = 2;
	float ks = 0.31;
	float kd = 0.62;
	//Make J
	vector<vector<float>> J(constraints.size(), vector<float>(particles.size() * n));
	for (int i = 0; i < constraints.size(); i++) {
		for (int j = 0; j < particles.size() * n; j++) {
			J[i][j] = 0;
		}
	}

	for (int i = 0; i < constraints.size(); i++) {
		vector<Vec2f> cons = constraints[i]->getJ();
		vector<Particle> part = constraints[i]->getParticles();
		for (int j = 0; j < part.size(); j++) {
			for (int o = 0; o < n; o++) {
				J[i][part[j].m_Number*n + o] = cons[j][o];
			}
		}
	}
	

	//Make W
	vector<vector<float>> W((particles.size() * n), vector<float>(particles.size() * n));
	for (int i = 0; i < particles.size() *n; i++) {
		for (int j = 0; j < particles.size()*n; j++) {
			W[i][j] = 0;
		}
	}

	for (int i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			W[n*i + o][n*i + o] = 1/particles[i]->m_Mass;
		}
	}

	//Make Jt
	vector< vector<float> > Jt((particles.size() * n), vector<float>(constraints.size()));
	for (int i = 0; i<constraints.size(); i++) {
		for (int j = 0; j < particles.size() * n; j++){
			Jt[j][i] = J[i][j];
		}
	}

	//Make Jdot
	vector<vector<float>> Jdot(constraints.size(), vector<float>(particles.size() * n));
	for (int i = 0; i < constraints.size(); i++) {
		for (int j = 0; j < particles.size() * n; j++) {
			Jdot[i][j] = 0;
		}
	}

	for (int i = 0; i < constraints.size(); i++) {
		vector<Vec2f> cons = constraints[i]->getJdot();
		vector<Particle> part = constraints[i]->getParticles();
		for (int j = 0; j < part.size(); j++) {
			for (int o = 0; o < n; o++) {
				Jdot[i][part[j].m_Number*n + o] = cons[j][o];
			}
		}
	}

	//Make qdot
	vector<float> qdot(particles.size()*n);
	for (int i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			qdot[n*i + o] = particles[i]->m_Velocity[o];
		}
	}

	//Make Q
	vector<float> Q(particles.size()*n);
	for (int i = 0; i < particles.size(); i++) {
		for (int o = 0; o < n; o++) {
			Q[n*i + o] = particles[i]->m_ForceAcc[o];
		}
	}

	//Make C
	vector<float> C(particles.size());
	for (int i = 0; i<constraints.size(); i++) {
		C[i] = constraints[i]->getC();
	}

	//Make Cdot
	vector<float> Cdot(particles.size());
	for (int i = 0; i<constraints.size(); i++) {
		Cdot[i] = constraints[i]->getCdot();
	}

	//Make JWJt and also JW
	vector<vector<float>> JWJt;
	vector<vector<float>> JW;
	JW = mul(J, W);
	JWJt = mul(JW, Jt);

	//Make −Jdot*qdot − JWQ with feedback
	vector<float> rightHandSide;
	rightHandSide = diffEqual(diffEqual(diffEqual(
		timesScalar(vecmul(Jdot, qdot), -1), vecmul(JW,Q))
		,timesScalar(C,ks))
		, timesScalar(Cdot, kd));


	//Solve
	implicitMatrix *M = new implicitMatrix(JWJt);
	//Should be lenth of constraints
	const int TODO = 2;
	double lambda[TODO], r[TODO];

	for (int i = 0; i < rightHandSide.size(); i++){
		r[i] = rightHandSide[i];
	}

	int d = 100;
	ConjGrad(TODO, M, lambda, r, 0.0000000000001, &d);
	delete M;

	//Make Qhat
	vector<float> fhat((constraints.size()));
	for (int i = 0; i < constraints.size(); i++){
		fhat[i] = lambda[i];
	}

	 vector<float> Qhat = vecmul(Jt, fhat);
	
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