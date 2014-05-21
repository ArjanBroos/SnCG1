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
	// Hook's law, IGNORE DAMPING FOR NOW
	Vec2f fp1 = ((float)m_ks * (nl - (float)m_dist) + (float)m_kd * (i * l) / nl) * (l / nl);
	m_p1->m_ForceAcc -= fp1;
	m_p2->m_ForceAcc += fp1;
}

// Returns a jacobian matrix for this force
Mat2 SpringForce::GetJacobian(int flags) const {
	Mat2 result;	// Will hold the requested Jacobian matrix for this force

	// Partial calculations
	const float x12 = m_p1->m_Position[0] - m_p2->m_Position[0];
	const float y12 = m_p1->m_Position[1] - m_p2->m_Position[1];
	const float x12s = x12*x12;
	const float y12s = y12*y12;
	const float ls = x12s + y12s;	// Distance squared
	const float l = sqrt(ls);		// Distance

	result[0][0] = (m_ks * (y12s - x12s) / ls) - (m_ks * m_dist * y12s) / (ls * l);		// df_x / dx1
	result[0][1] = (m_ks * m_dist * y12) / (ls * l) + ((x12 - 1.f) * m_ks * y12) / ls;	// df_x / dy1
	result[1][0] = (m_ks * m_dist * x12) / (ls * l) + ((y12 - 1.f) * m_ks * x12) / ls;	// df_y / dx1
	result[1][1] = (m_ks * (x12s - y12s) / ls) - (m_ks * m_dist * x12s) / (ls * l);		// df_y / dy1

	return result;
}

/*
Mat2 SpringForce::GetJacobian(int flags) const {
	const Vec2f xij = m_p1->m_Position - m_p2->m_Position;
	const float l = norm(xij);
	const Vec2f xijn = xij / l;
	
	Mat2 mxijn;
	mxijn[0][0] = xijn[0] * xijn[0]; mxijn[0][1] = xijn[0] * xijn[1];
	mxijn[1][0] = xijn[1] * xijn[0]; mxijn[1][1] = xijn[1] * xijn[1];

	Mat2 I;
	I[0][0] = 1.f; I[0][1] = 0.f;
	I[1][0] = 0.f; I[1][1] = 1.f;

	Mat2 jac = -m_ks * ( (1.0 - m_dist / l) * (I - mxijn) + mxijn );
	return jac;
}*/

// Return the index of particle 1
 unsigned SpringForce::GetP1Index() const {
	 return m_p1->m_Number;
 }

// Return the index of particle 2
unsigned SpringForce::GetP2Index() const {
	return m_p2->m_Number;
}