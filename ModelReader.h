#pragma once

#include <string>
#include "ParticleSystem.h"
#include <fstream>
#include <vector>
#include "Particle.h"
#include "SpringForce.h"
#include "GravityForce.h"
#include "ViscousDragForce.h"

// Helper class to read vertices and edges from a file
// and use them as particles and springs
class ModelReader {
public:
	ModelReader(float ks, float kd, bool gravity, bool drag, float kdrag = 0.f) :
		ks(ks), kd(kd), gravity(gravity), drag(drag), kdrag(kdrag) {}
	void ReadModel(const std::string& filename, ParticleSystem& ps);

private:
	void SkipCommentLine(std::ifstream& file);
	void ReadInfo(std::ifstream& file);
	void ReadParticle(std::ifstream& file);
	void ReadSpring(std::ifstream& file);

	float						ks;			// Stiffness constant
	float						kd;			// Damping constant
	float						kdrag;		// Viscous drag constant
	bool						gravity;	// Add gravity to particles?
	bool						drag;		// Add viscous drag to particles?

	std::vector<Particle*>		p;			// Particles
	std::vector<SpringForce*>	sf;		// Spring forces
};