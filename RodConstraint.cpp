#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::Draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

// return the c: C(x1, y1, x2, y2) = (x1 - x2)^2 + (y1 - y2)^2 - r^2
double RodConstraint::getC(){
	return (m_p1->m_Position[0] - m_p2->m_Position[0])*(m_p1->m_Position[0] - m_p2->m_Position[0]) + (m_p1->m_Position[1] - m_p2->m_Position[1])*(m_p1->m_Position[1] - m_p2->m_Position[1]) - m_dist*m_dist;
}

// return the cdot: Cdot(x, y) = 
double RodConstraint::getCdot(){
	return 2 * (((m_p1->m_Position[0] - m_p2->m_Position[0])*(m_p1->m_Velocity[0]) + (m_p1->m_Position[1] - m_p2->m_Position[1])*(m_p1->m_Velocity[1])) - ((m_p1->m_Position[0] - m_p2->m_Position[0])*(m_p2->m_Velocity[0]) + (m_p1->m_Position[1] - m_p2->m_Position[1])*(m_p2->m_Velocity[1])));
}

//return J, if there are more use same order as particle
vector<Vec2f> RodConstraint::getJ(){
	vector<Vec2f> result;
	result.push_back(Vec2f( 2 * (m_p1->m_Position[0] - m_p2->m_Position[0]), 2 * (m_p1->m_Position[1] - m_p2->m_Position[1]) ));
	result.push_back(Vec2f( 2 * (m_p2->m_Position[0] - m_p1->m_Position[0]), 2 * (m_p2->m_Position[1] - m_p1->m_Position[1]) ));
	return result;
}

//return Jdot, if there are more use same order as particle
vector<Vec2f> RodConstraint::getJdot(){
	vector<Vec2f> result;
	result.push_back(Vec2f( 2 * (m_p1->m_Velocity[0] - m_p2->m_Velocity[0]), 2 * (m_p1->m_Velocity[1] - m_p2->m_Velocity[1]) ));
	result.push_back(Vec2f( 2 * (m_p2->m_Velocity[0] - m_p1->m_Velocity[0]), 2 * (m_p2->m_Velocity[1] - m_p1->m_Velocity[1]) ));
	return result;
}
vector<Particle> RodConstraint::getParticles(){
	vector<Particle> result;
	result.push_back(*m_p1);
	result.push_back(*m_p2);
	return result;
}