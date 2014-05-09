#include "linearSolver.h"
using namespace std;

// vector helper functions

void vecAddEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] + v[i];
}

void vecDiffEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] - v[i];
}

void vecAssign(int n, double v1[], double v2[])
{
  for (int i = 0; i < n; i++)
    v1[i] = v2[i];
}

void vecTimesScalar(int n, double v[], double s)
{
  for (int i = 0; i < n; i++)
    v[i] *= s;
}

double vecDot(int n, double v1[], double v2[])
{
  double dot = 0;
  for (int i = 0; i < n; i++)
    dot += v1[i] * v2[i];
  return dot;
}

double vecSqrLen(int n, double v[])
{
  return vecDot(n, v, v);
}

double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps)
{
  int		i, iMax;
  double	alpha, beta, rSqrLen, rSqrLenOld, u;

  double *r = (double *) malloc(sizeof(double) * n);
  double *d = (double *) malloc(sizeof(double) * n);
  double *t = (double *) malloc(sizeof(double) * n);
  double *temp = (double *) malloc(sizeof(double) * n);

  vecAssign(n, x, b);

  vecAssign(n, r, b);
  A->matVecMult(x, temp);
  vecDiffEqual(n, r, temp);

  rSqrLen = vecSqrLen(n, r);

  vecAssign(n, d, r);

  i = 0;
  if (*steps)
    iMax = *steps;
  else
    iMax = MAX_STEPS;
		
  if (rSqrLen > epsilon)
    while (i < iMax) {	
      i++;
      A->matVecMult(d, t);
      u = vecDot(n, d, t);
      
      if (u == 0) {
	printf("(SolveConjGrad) d'Ad = 0\n");
	break;
      }
      
      // How far should we go?
      alpha = rSqrLen / u;
      
      // Take a step along direction d
      vecAssign(n, temp, d);
      vecTimesScalar(n, temp, alpha);
      vecAddEqual(n, x, temp);
      
      if (i & 0x3F) {
	vecAssign(n, temp, t);
	vecTimesScalar(n, temp, alpha);
	vecDiffEqual(n, r, temp);
      } else {
	// For stability, correct r every 64th iteration
	vecAssign(n, r, b);
	A->matVecMult(x, temp);
	vecDiffEqual(n, r, temp);
      }
      
      rSqrLenOld = rSqrLen;
      rSqrLen = vecSqrLen(n, r);
      
      // Converged! Let's get out of here
      if (rSqrLen <= epsilon)
	break;			    
      
      // Change direction: d = r + beta * d
      beta = rSqrLen/rSqrLenOld;
      vecTimesScalar(n, d, beta);
      vecAddEqual(n, d, r);
    }
  
  // free memory

  free(r);
  free(d);
  free(t);
  free(temp);
		
  *steps = i;
  return(rSqrLen);
}


//Matrix helper functions

/*
* Return A*B
*/
vector< vector<float> > mul(vector< vector<float> > A, vector< vector<float> > B) {

	if (A[0].size() != B.size()) {
		printf("Error: Matrix multiplication dimension does not match! \n");
		exit(1); //error!
	}
	vector< vector<float> > R(A.size(), vector<float>(B[0].size()));

	for (int i = 0; i<A.size(); i++) {
		for (int j = 0; j<B[0].size(); j++) {
			R[i][j] = 0;
			for (int k = 0; k<A[0].size(); k++)
				R[i][j] += A[i][k] * B[k][j];
		}
	}

	return R;
}

/*
* Return A*b(b is a vector, not a matrix)
*/
 vector<float>  vecmul(vector< vector<float> > A, vector<float>  B) {

	if (A[0].size() != B.size()) {
		printf("Error: Matrix multiplication dimension does not match! \n");
		exit(1); //error!
	}
	vector<float>  R(A.size());

	for (int i = 0; i<A.size(); i++) {
		R[i] = 0;
		for (int k = 0; k<B.size(); k++)
			R[i] += A[i][k] * B[k];
	}
	

	return R;
}

 vector<float> diffEqual(vector<float> r, vector<float> v)
 {
	 vector<float> result(r.size());
	 for (int i = 0; i < r.size(); i++){
		 result[i] = r[i] - v[i];
	 }
	 return result;
 }

 vector<float> timesScalar(vector<float> r, float s)
 {
	 vector<float> result(r.size());
	 for (int i = 0; i < r.size(); i++){
		 result[i] = r[i] * s;
	 }
	 return result;
 }

 