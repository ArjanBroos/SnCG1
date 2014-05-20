#include "GravityForce.h"
#include <GL/glut.h>

GravityForce::GravityForce(Particle* particle) {
	this->particle = particle;
}

// Draw the force on the associated particle
void GravityForce::Draw() const {
	glBegin(GL_LINES);
		glColor3f(0.2f, 0.2f, 0.4f);
		glVertex2f(particle->m_Position[0], particle->m_Position[1]);
		glVertex2f(particle->m_Position[0], particle->m_Position[1] - 0.08f);
	glEnd();
}

// Apply the force to the associated particle
void GravityForce::Apply() {
	const float G = 9.81;	// Gravitational constant
	particle->m_ForceAcc[1] -= particle->m_Mass * G;
}

// Returns a jacobian matrix for this force (all 0)
Mat2 GravityForce::GetJacobian(int flags) const {
	// Gravity force is not dependent on position or velocity
	return Mat2(0.f, 0.f, 0.f, 0.f);
}