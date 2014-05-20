#include "SpringForce.h"
#include <GL/glut.h>
#include <iostream>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

// Draw the force between the associated particles
void SpringForce::Draw() const {
  glBegin( GL_LINES );
	  glColor3f(0.6, 0.7, 0.8);
	  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
	  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

// Apply the force to the associated particles
void SpringForce::Apply() {
	Vec2f l = m_p1->m_Position - m_p2->m_Position;
	Vec2f i = m_p1->m_Velocity - m_p2->m_Velocity;

	float nl = norm(l);
	// Hook's law
	Vec2f fp1 = (float)(m_ks * (nl - m_dist) + m_kd * (i*l) / nl) * (l / nl);
	m_p1->m_ForceAcc -= fp1;
	m_p2->m_ForceAcc += fp1;
}

// Returns a jacobian matrix for this force
// Use the flags parameter to specify which particle
// Also use the flags parameter to specify with respect to what we calculate this matrix (position or velocity)
// For example: GetJacobian(JF_P1 | JF_POS) will return the jacobian matrix for particle #1, wrt position
Mat2 SpringForce::GetJacobian(int flags) const {
	Mat2 result;	// Will hold the requested Jacobian matrix for this force

	// Shorter aliases
	const float& x1 = m_p1->m_Position[0];
	const float& x2 = m_p2->m_Position[0];
	const float& y1 = m_p1->m_Position[1];
	const float& y2 = m_p2->m_Position[1];
	const float& vx1 = m_p1->m_Velocity[0];
	const float& vx2 = m_p2->m_Velocity[0];
	const float& vy1 = m_p1->m_Velocity[1];
	const float& vy2 = m_p2->m_Velocity[1];

	// Partial calculations
	const float x12 = x1 - x2;
	const float y12 = y1 - y2;
	const float x12s = x12*x12;
	const float y12s = y12*y12;
	const float ls = x12s + y12s;	// Distance squared
	const float l = sqrt(ls);		// Distance
	const float vx12 = vx1 - vx2;
	const float vy12 = vy1 - vy2;

	if (flags & JF_POS) { // Jacobian wrt position
		result[0][0] = m_ks * (x12 - m_dist) * (-x12 / (ls * l)) + m_ks * (x12 / l);	// df_x / dx1
		result[0][1] = (m_kd * vx12 * -2.f * x12s * y12) / (ls * ls);					// df_x / dy1
		result[1][0] = (m_kd * vy12 * -2.f * y12s * x12) / (ls * ls);					// df_y / dx1
		result[1][1] = m_ks * (y12 - m_dist) * (-y12 / (ls * l)) + m_ks * (y12 / l);	// df_y / dy1
	} else if (flags & JF_VEL) { // Jacobian wrt velocity
		result[0][0] = (m_kd * x12s) / ls;	// df_x / dvx1
		result[0][1] = 0.f;					// df_x / dvy1
		result[1][0] = 0.f;					// df_y / dvx1
		result[1][1] = (m_kd * y12s) / ls;	// df_y / dvy1
	} else {
		std::cerr << "Wrong flags in GetJacobian for spring force (pos vs vel)" << std::endl;
	}

	if (flags & JF_P1) { // For particle 1
		return result;
	} else if (flags & JF_P2) { // For particle 2
		return -result;
	} else {
		std::cerr << "Wrong flags in GetJacobian for spring force (p1 vs p2)" << std::endl;
		return result;
	}
}

// Return the index of particle 1
 unsigned SpringForce::GetP1Index() const {
	 return m_p1->m_Number;
 }

// Return the index of particle 2
unsigned SpringForce::GetP2Index() const {
	return m_p2->m_Number;
}