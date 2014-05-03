#pragma once

// Abstract base class for constraints
class Constraint {
public:
	// Every constraint will need to implement these functions

	// Applies this constraint to its associated particles
	virtual void Apply() = 0;
	// Draws the constraint between the associated particles
	virtual void Draw() const = 0;
};