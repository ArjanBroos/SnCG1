#include "ViscousDragForce.h"
#include <GL\glut.h>

ViscousDragForce::ViscousDragForce(Particle* p, float drag) : particle(p), drag(drag) {
}

// Applies this force to its associated particles
void ViscousDragForce::Apply() {
	particle->m_ForceAcc -= drag * particle->m_Velocity;
}

// Draws the force between the associated particles
void ViscousDragForce::Draw() const {
	glBegin(GL_LINES);
		glColor3f(0.4f, 0.2f, 0.2f);
		glVertex2f(particle->m_Position[0], particle->m_Position[1]);
		glVertex2f(particle->m_Position[0] - drag * 0.2f * particle->m_Velocity[0],
			particle->m_Position[1] - drag * 0.2f * particle->m_Velocity[1]);
	glEnd();
}