#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::Draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

// return the c: C(x1, y1, x2, y2) = (x1 - x2)^2 + (y1 - y2)^2 - r^2
double RodConstraint::getC(){
	const float& x1 = m_p1->m_Position[0];
	const float& x2 = m_p2->m_Position[0];
	const float& y1 = m_p1->m_Position[1];
	const float& y2 = m_p2->m_Position[1];
	return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) - m_dist*m_dist;
}

// return the cdot: Cdot(x, y , vx, vy) = (x-y)*vx - ((x-y)*vy)
double RodConstraint::getCdot(){
	const float& x1 = m_p1->m_Position[0];
	const float& x2 = m_p2->m_Position[0];
	const float& y1 = m_p1->m_Position[1];
	const float& y2 = m_p2->m_Position[1];
	const float& vx1 = m_p1->m_Velocity[0];
	const float& vx2 = m_p2->m_Velocity[0];
	const float& vy1 = m_p1->m_Velocity[1];
	const float& vy2 = m_p2->m_Velocity[1];
	return ((x1 - x2)*(vx1) + (y1 - y2)*(vy1)) - ((x1 - x2)*(vx2) + (y1 - y2)*(vy2));
}

//return J, if there are more use same order as particle
vector<Vec2f> RodConstraint::getJ(){
	vector<Vec2f> result;
	result.push_back(2.f * (m_p1->m_Position - m_p2->m_Position));
	result.push_back(2.f * (m_p2->m_Position - m_p1->m_Position));
	return result;
}

//return Jdot, if there are more use same order as particle
vector<Vec2f> RodConstraint::getJdot(){
	vector<Vec2f> result;
	result.push_back(2.f * (m_p1->m_Velocity - m_p2->m_Velocity));
	result.push_back(2.f * (m_p2->m_Velocity - m_p1->m_Velocity));
	return result;
}
vector<Particle> RodConstraint::getParticles(){
	vector<Particle> result;
	result.push_back(*m_p1);
	result.push_back(*m_p2);
	return result;
}