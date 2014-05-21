#include "LineConstraint.h"
#include <GL/glut.h>

LineConstraint::LineConstraint(Particle *p1, Vec2f start, Vec2f direction) :
  m_p1(p1), m_start(start), m_direction(direction) {}

void LineConstraint::Draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_start[0]-m_direction[0]*2, m_start[1]-m_direction[1]*2 );
  glVertex2f( m_start[0]+m_direction[0]*2, m_start[1]+m_direction[1]*2 );
  glEnd();
}

// return the c: C(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
double LineConstraint::getC(){
	const float x = -m_direction[1]-m_start[0];
	const float y = m_direction[0]-m_start[1];
	Vec2f perp = Vec2f(x,y);
	norm(perp);
	return perp*(m_p1->m_Position-m_start);
}

// return the cdot: Cdot(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
double LineConstraint::getCdot(){
	Vec2f perp = Vec2f(-m_direction[1]-m_start[0],m_direction[0]-m_start[1]);
	norm(perp);
	return perp*(m_p1->m_Velocity-m_start);
}

//return J, if there are more use same order as particle
//(-dy - sx),(dx-sy)).norm * 1
vector<Vec2f> LineConstraint::getJ(){
	vector<Vec2f> result;
	Vec2f perp = Vec2f(-m_direction[1]-m_start[0],m_direction[0]-m_start[1]);
	norm(perp);
	result.push_back(perp);
	return result;
}

//return Jdot, if there are more use same order as particle
vector<Vec2f> LineConstraint::getJdot(){
	vector<Vec2f> result;
	result.push_back(Vec2f(0,0));
	return result;
}
vector<Particle> LineConstraint::getParticles(){
	vector<Particle> result;
	result.push_back(*m_p1);
	return result;
}