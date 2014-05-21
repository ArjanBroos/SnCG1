#pragma once

#include "Particle.h"
#include <vector>

class CollidableLineSegment {
 public:
  CollidableLineSegment(Vec2f start, Vec2f end, double k, double t);

  void Draw() const;
  void handleCollision(Particle *p1);
  void handleCollisions(std::vector<Particle*>& p1);

 private:

  Vec2f m_start;
  Vec2f m_end;
  double m_k; // bounciness
  double m_t; // how close for collision: how many times the current velocity?
};
