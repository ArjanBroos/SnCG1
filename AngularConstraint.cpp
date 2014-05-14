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

// return the cdot: Cdot(x, y) = |v_r|sin/|r| met sin=sqrt(1-cos^2) en cos = dot /(|v_r||r|) , r/in{a,b}
double AngularConstraint::getCdot(){
	double* r = new double[2];
	r[0] = m_joint->m_Position[0] - m_p1->m_Position[0];
	r[1] = m_joint->m_Position[1] - m_p1->m_Position[1];
	double* v = new double[2];
	v[0] = m_p1->m_Velocity[0] - m_joint->m_Velocity[0];
	v[1] = m_p1->m_Velocity[1] - m_joint->m_Velocity[1];
	double angspeed1 = 0;
	if (sqrt(v[0] * v[0] + v[1] * v[1])!=0){
		double cos = vecDot(2, r, v) / (sqrt(v[0] * v[0] + v[1] * v[1])*sqrt(r[0] * r[0] + r[1] * r[1]));
		angspeed1 = (sqrt(v[0] * v[0] + v[1] * v[1])*sin(sqrt(1 - cos*cos)) )/ sqrt(r[0] * r[0] + r[1] * r[1]);
	}

	r[0] = m_joint->m_Position[0] - m_p2->m_Position[0];
	r[1] = m_joint->m_Position[1] - m_p2->m_Position[1];
	v[0] = m_p2->m_Velocity[0] - m_joint->m_Velocity[0];
	v[1] = m_p2->m_Velocity[1] - m_joint->m_Velocity[1];
	double angspeed2 = 0;
	if (sqrt(v[0] * v[0] + v[1] * v[1]) != 0){
		double cos = vecDot(2, r, v) / (sqrt(v[0] * v[0] + v[1] * v[1])*sqrt(r[0] * r[0] + r[1] * r[1]));
		angspeed2 = (sqrt(v[0] * v[0] + v[1] * v[1])*sin(sqrt(1 - cos*cos)) )/ sqrt(r[0] * r[0] + r[1] * r[1]);
	}

	printf("%f  \n", angspeed1 - angspeed2);
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