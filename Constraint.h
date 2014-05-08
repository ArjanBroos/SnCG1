#pragma once
#include <vector>
using namespace std;

// Abstract base class for constraints
class Constraint {
public:
	// Every constraint will need to implement these functions

	// Draws the constraint between the associated particles
	virtual void Draw() const = 0;

	virtual double getC() = 0;
	virtual double getCdot() = 0 ;
	virtual vector<Vec2f> getJ() = 0;
	virtual vector<Vec2f> getJdot() = 0;
	virtual vector<Particle> getParticles() = 0;
};