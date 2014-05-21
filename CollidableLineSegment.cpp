#include "CollidableLineSegment.h"
#include <GL/glut.h>

CollidableLineSegment::CollidableLineSegment(Vec2f start, Vec2f end, double k, double t) :
  m_start(start), m_end(end), m_k(k), m_t(t) {}

void CollidableLineSegment::Draw() const
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_start[0], m_start[1] );
  glVertex2f( m_end[0],   m_end[1] );
  glEnd();
}

// return the c: C(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
void CollidableLineSegment::handleCollision(Particle *p1){
	Vec2f perp = Vec2f(m_end[1]-m_start[1],-(m_end[0]-m_start[0]));
	norm(perp);
	double projPN = p1->m_Velocity*perp;

	double dist = perp*(p1->m_Position-m_start);
	double times = dist/projPN; // times the velocity is needed to reach line
	
	if (times <= m_t && times>0){ //potential collision (close enough)
		
		Vec2f colP = Vec2f(p1->m_Position-(float)times*(p1->m_Velocity)); // point of collision (line-line intersection)
		double x = (colP[1]-m_start[1])/(m_end[1]-m_start[1]);
		if ((x>=0) && (x<=1)){
			//Truly colliding (direction and within line
			Vec2f pn = Vec2f (projPN)*perp;
			Vec2f pt = p1->m_Velocity - pn;

			p1->m_Velocity = pt-(float)m_k*pn;
		}
	}
}

// return the c: C(x, y, sx, sy, dx, dy) = ((-dy - sx),(dx-sy)).norm * ((x,y)-(sx,sy))
void CollidableLineSegment::handleCollisions(std::vector<Particle*>& p1){
	for each (Particle* p in p1){
		handleCollision(p);
	}
	//Vec2f dir = Vec2f(m_direction-m_start);
	//norm(dir);
	//double proj = dir*(m_p1->m_Position-m_start);
	//Vec2f lineproj = (float)proj*m_direction + m_start;
	//return (lineproj - m_p1->m_Position)*(lineproj - m_p1->m_Position);
}