#include "ViscousDragForce.h"
#include <GL\glut.h>

ViscousDragForce::ViscousDragForce(Particle* p, float drag) : particle(p), drag(drag) {
}

// Applies this force to its associated particle
void ViscousDragForce::Apply() {
	particle->m_ForceAcc -= drag * particle->m_Velocity;
}

// Draws the force on the associated particle
void ViscousDragForce::Draw() const {
	glBegin(GL_LINES);
		glColor3f(0.4f, 0.2f, 0.2f);
		glVertex2f(particle->m_Position[0], particle->m_Position[1]);
		glVertex2f(particle->m_Position[0] - drag * 0.2f * particle->m_Velocity[0],
			particle->m_Position[1] - drag * 0.2f * particle->m_Velocity[1]);
	glEnd();
}

// Returns a jacobian matrix for this force wrt velocity
Mat2 ViscousDragForce::GetJacobian(int flags) const {
	return Mat2(-drag, 0.f, 0.f, -drag);
}

// Returns the index of the particle
unsigned ViscousDragForce::GetPIndex() const {
	return particle->m_Number;
}