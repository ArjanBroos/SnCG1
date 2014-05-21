#pragma once

#include "Particle.h"
#include "Constraint.h"

class CollidableLineSegment {
 public:
  CollidableLineSegment(Vec2f start, Vec2f end);

  void Draw() const;
  void handleCollision(Particle *p1);

 private:

  Vec2f m_start;
  Vec2f m_end;
};
