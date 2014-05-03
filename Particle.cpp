#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos),
	m_Position(ConstructPos),
	m_Velocity(Vec2f(0.f, 0.f)),
	m_Mass(0.1f),
	m_ForceAcc(Vec2f(0.f, 0.f))
{
}

Particle::~Particle(void)
{
}

void Particle::Reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.f, 0.f);
	m_ForceAcc = Vec2f(0.f, 0.f);
}
void Particle::Draw() const
{
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
		glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
		glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
		glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
		glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}
