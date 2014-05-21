#include "SpringAngleForce.h"
#include <GL/glut.h>
#include <iostream>

SpringAngleForce::SpringAngleForce(Particle *p1, Particle *joint, Particle * p2, double angle, double ks, double kd) :
m_p1(p1), m_p2(p2), m_joint(joint), m_angle(angle), m_ks(ks), m_kd(kd) {}

// Draw the force between the associated particles
void SpringAngleForce::Draw() const {
	float radius = 0.1;
	Vec2f a = m_p1->m_Position - m_joint->m_Position;
	Vec2f b = m_p2->m_Position - m_joint->m_Position;
	a = (a / sqrt(a*a));
	b = (b / sqrt(b*b));
	a = Vec2f(m_joint->m_Position[0] + a[0] * radius, m_joint->m_Position[1] + a[1] * radius);
	b = Vec2f(m_joint->m_Position[0] + b[0] * radius, m_joint->m_Position[1] + b[1] * radius);

	glBegin(GL_LINES);
	glColor3f(0.8, 0.0, 0.0);
	glVertex2f(a[0], a[1]);
	glVertex2f(b[0], b[1]);
	glEnd();
}

// Apply the force to the associated particles
void SpringAngleForce::Apply() {
	Vec2f a = m_p1->m_Position - m_joint->m_Position;
	float length = sqrt(a[0] * a[0] + a[1] * a[1]);
	a /= length;
	Vec2f b = m_p2->m_Position - m_joint->m_Position;
	length = sqrt(b[0] * b[0] + b[1] * b[1]);
	b /= length;
	float restangle = acos(a*b);
	if ((a[0] * b[1] - a[1] * b[0])>0){
		restangle = 2 * M_PI - restangle;
	}
	restangle -= m_angle;
	if (restangle>M_PI){
		restangle = ((restangle)-2 * M_PI);
	}
	m_p1->m_ForceAcc = 0;
	m_p2->m_ForceAcc = 0;

}

Mat2 SpringAngleForce::GetJacobian(int flags) const {
	Mat2 result;
	return result;
}