#pragma once

#include "Force.h"
#include "Particle.h"

// A force that models gravity
class GravityForce : public Force {
public:
	GravityForce(Particle* particle);

	void		Draw() const;
	void		Apply();

private:
	Particle*	particle;
};