#include "AngularConstraint.h"
#include "linearSolver.h"
#include <GL/glut.h>

AngularConstraint::AngularConstraint(Particle *p1, Particle * pjoint, Particle * p2, double angle) :
m_p1(p1), m_p2(p2), m_joint(pjoint),m_angle(angle) {}

void AngularConstraint::Draw() const
{
	float radius = 0.1;
	Vec2f a = m_p1->m_Position - m_joint->m_Position;
	Vec2f b = m_p2->m_Position - m_joint->m_Position;
	a = (a / sqrt(a*a));
	b = (b / sqrt(b*b));
	a = { m_joint->m_Position[0] + a[0] * radius , m_joint->m_Position[1] + a[1] * radius };
	b = { m_joint->m_Position[0] + b[0] * radius, m_joint->m_Position[1] + b[1] * radius };
	
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( a[0],a[1]);
  glVertex2f( b[0], b[1]);
  glEnd();
}

// return the c: C(x1, y1, xj, yj, x2, y2) = a dot b - |a|*|b|* cos(angle)
double AngularConstraint::getC(){
	double* a = new double[2];
	a[0] = m_p1->m_Position[0] - m_joint->m_Position[0];
	a[1] = m_p1->m_Position[1] - m_joint->m_Position[1];
	double* b = new double[2];
	b[0] = m_p2->m_Position[0] - m_joint->m_Position[0];
	b[1] = m_p2->m_Position[1] - m_joint->m_Position[1];
	return 0;// vecDot(2, a, b) - sqrt(a[0] * a[0] + a[1] * a[1])*sqrt(b[0] * b[0] + b[1] * b[1])* cos(m_angle);
}

// return the cdot: Cdot(x, y) = |v|sin/|r| met sin=sqrt(1-cos^2) en cos = dot /(|v||r|)
double AngularConstraint::getCdot(){
	double* a = new double[2];
	a[0] = m_joint->m_Position[0] - m_p1->m_Position[0];
	a[1] = m_joint->m_Position[1] - m_p1->m_Position[1];
	double* b = new double[2];
	b[0] = a[0] + m_p1->m_Velocity[0] - m_joint->m_Velocity[0];
	b[1] = a[1] + m_p1->m_Velocity[1] - m_joint->m_Velocity[1];
	double da = vecDot(2, a, b);

	a[0] = m_joint->m_Position[0] - m_p2->m_Position[0];
	a[1] = m_joint->m_Position[1] - m_p2->m_Position[1];
	b[0] = a[0] + m_p2->m_Velocity[0] - m_joint->m_Velocity[0];
	b[1] = a[1] + m_p2->m_Velocity[1] - m_joint->m_Velocity[1];
	double db = vecDot(2, a, b);
	printf("%f \t %f \n",da,db);
	return 0;
}

//return J, if there are more use same order as particle
vector<Vec2f> AngularConstraint::getJ(){
	vector<Vec2f> result;
	return result;
}

//return Jdot, if there are more use same order as particle
vector<Vec2f> AngularConstraint::getJdot(){
	vector<Vec2f> result;
	return result;
}
vector<Particle> AngularConstraint::getParticles(){
	vector<Particle> result;
	//result.push_back(*m_p1);
	//result.push_back(*m_p2);
	//result.push_back(*m_joint);
	return result;
}