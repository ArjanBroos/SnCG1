#include "GravityForce.h"
#include <GL/glut.h>

GravityForce::GravityForce(Particle* particle) {
	this->particle = particle;
}

void GravityForce::Draw() const {
	glBegin(GL_LINES);
		glColor3f(0.2f, 0.2f, 0.4f);
		glVertex2f(particle->m_Position[0], particle->m_Position[1]);
		glVertex2f(particle->m_Position[0], particle->m_Position[1] - 0.08f);
	glEnd();
}

void GravityForce::Apply() {
	const float G = 9.81;	// Gravitational constant
	particle->m_ForceAcc[1] -= particle->m_Mass * G;
}