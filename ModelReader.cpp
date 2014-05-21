#include "ModelReader.h"

#include <iostream>

void ModelReader::ReadModel(const std::string& filename, ParticleSystem& ps) {
	std::ifstream file(filename.c_str());

	if (!file.is_open()) {
		std::cerr << "Could not open " << filename << std::endl;
		return;
	}

	while (!file.eof()) {
		char identifier;
		file >> identifier;
		if (identifier == '#')		SkipCommentLine(file);
		else if (identifier == 'i') ReadInfo(file);
		else if (identifier == 'v') ReadParticle(file);
		else if (identifier == 'e') ReadSpring(file);
		else { std::cerr << "Wrong identifier in " << filename << std::endl; return; }
	}

	for (auto i = p.begin(); i != p.end(); i++)
		ps.AddParticle(*i);
	for (auto i = sf.begin(); i != sf.end(); i++)
		ps.AddForce(*i);
	if (gravity) {
		for (auto i = p.begin(); i != p.end(); i++)
			ps.AddForce(new GravityForce(*i));
	}
	if (drag) {
		for (auto i = p.begin(); i != p.end(); i++)
			ps.AddForce(new ViscousDragForce(*i, kdrag));
	}
}

void ModelReader::SkipCommentLine(std::ifstream& file) {
	std::string trash;
	std::getline(file, trash);
}

void ModelReader::ReadInfo(std::ifstream& file) {
	unsigned nrParticles;
	file >> nrParticles;
	p.resize(nrParticles, nullptr);
}

void ModelReader::ReadParticle(std::ifstream& file) {
	float x, y;
	unsigned index;
	file >> x;
	file >> y;
	file >> index;
	p[index] = new Particle(Vec2f(x, y));
}

void ModelReader::ReadSpring(std::ifstream& file) {
	unsigned index1, index2;
	file >> index1;
	file >> index2;
	float distance = norm(p[index1]->m_Position - p[index2]->m_Position);
	sf.push_back(new SpringForce(p[index1], p[index2], distance, ks, kd));
}
