#include "LineConstraint.h"
#include <GL/glut.h>

LineConstraint::LineConstraint(Particle *p1, Vec2f start, Vec2f direction) :
  m_p1(p1), m_start(start), m_direction(direction) {}

void LineConstraint::Draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_start[0]-m_direction[0]*2, m_start[1]-m_direction[1]*4 );
  glVertex2f( m_start[0]+m_direction[0]*2, m_start[1]+m_direction[1]*4 );
  glEnd();
}

// return the c: C(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
double LineConstraint::getC(){
	//Vec2f dir = Vec2f(m_direction-m_start);
	//norm(dir);
	//double proj = dir*(m_p1->m_Position-m_start);
	//Vec2f lineproj = (float)proj*m_direction + m_start;
	//return (lineproj - m_p1->m_Position)*(lineproj - m_p1->m_Position);
	return m_p1->m_Position[1]-m_start[1];
}

// return the cdot: Cdot(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
double LineConstraint::getCdot(){
	//Vec2f dir = Vec2f(m_direction-m_start);
	//norm(dir);
	//double proj = dir*(m_p1->m_Position-m_start);
	//Vec2f lineproj = m_direction*(float)proj + m_start;
	//return (lineproj - m_p1->m_Position)*m_p1->m_Velocity;
	return m_p1->m_Velocity[1];
}

//return J, if there are more use same order as particle
//(-dy - sx),(dx-sy)).norm * 1
vector<Vec2f> LineConstraint::getJ(){
	vector<Vec2f> result;
	//Vec2f dir = Vec2f(m_direction-m_start);
	//norm(dir);
	//double proj = dir*(m_p1->m_Position-m_start);
	//Vec2f lineproj = m_direction*(float)proj + m_start;
	//result.push_back(2.f*(m_p1->m_Position-lineproj));

	result.push_back(Vec2f(0,1));

	return result;

}

//return Jdot, if there are more use same order as particle
vector<Vec2f> LineConstraint::getJdot(){
	vector<Vec2f> result;
	//Vec2f dir = Vec2f(m_direction-m_start);
	//norm(dir);
	//double proj = dir*(m_p1->m_Position-m_start);
	//Vec2f lineproj = m_direction*(float)proj + m_start;
	//result.push_back(2.f*(m_p1->m_Velocity-lineproj));

	result.push_back(Vec2f(0,0));

	return result;
}
vector<Particle> LineConstraint::getParticles(){
	vector<Particle> result;
	result.push_back(*m_p1);
	return result;
}


Vec2f LineConstraint::normalPerpendicularVector(){
	Vec2f result = Vec2f((m_direction[1]-m_start[1]),-(m_direction[0]-m_start[0]));
	norm(result);
	return result;
}