#pragma once

#include <math.h> 
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <vector>
using namespace std;

// Karen's CGD

#define MAX_STEPS 100

// Matrix class the solver will accept
class implicitMatrix
{
public: 
	implicitMatrix(vector< vector<float> > x) {
		matrix = x;
	}

	virtual void matVecMult(double x[], double r[]) {
		for (int i = 0; i<matrix.size(); i++) {
			r[i] = 0;
			for (int j = 0; j < matrix[0].size(); j++){
				r[i] += matrix[i][j] * x[j];
			}
		}
	}

private:
	vector< vector<float> > matrix;
};

// Matrix class the solver will accept
class implicitMatrixWithTrans : public implicitMatrix
{
 public:
  virtual void matVecMult(double x[], double r[]) = 0;
  virtual void matTransVecMult(double x[], double r[]) = 0;
};



// Solve Ax = b for a symmetric, positive definite matrix A
// A is represented implicitely by the function "matVecMult"
// which performs a matrix vector multiple Av and places result in r
// "n" is the length of the vectors x and b
// "epsilon" is the error tolerance
// "steps", as passed, is the maximum number of steps, or 0 (implying MAX_STEPS)
// Upon completion, "steps" contains the number of iterations taken
double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps);

// Some vector helper functions
void vecAddEqual(int n, double r[], double v[]);
void vecDiffEqual(int n, double r[], double v[]);
void vecAssign(int n, double v1[], double v2[]);
void vecTimesScalar(int n, double v[], double s);
double vecDot(int n, double v1[], double v2[]);
double vecSqrLen(int n, double v[]);
// Some matrix helper functions
vector<vector<float>> mul(vector< vector<float> > A, vector< vector<float> > B);
vector<float> vecmul(vector< vector<float> > A,  vector<float>  B);
vector<float> diffEqual(vector<float> r, vector<float> v);
vector<float> timesScalar(vector<float> r, float s);


