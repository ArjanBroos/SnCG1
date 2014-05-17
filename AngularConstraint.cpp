#include "AngularConstraint.h"
#include "linearSolver.h"
#include <GL/glut.h>

AngularConstraint::AngularConstraint(Particle *p1, Particle * pjoint, Particle * p2, double angle) :
m_p1(p1), m_p2(p2), m_joint(pjoint),m_angle(angle) {}

void AngularConstraint::Draw() const
{
	float radius = 0.1;
	Vec2f a = m_p1->m_Position - m_joint->m_Position;
	Vec2f b = m_p2->m_Position - m_joint->m_Position;
	a = (a / sqrt(a*a));
	b = (b / sqrt(b*b));
	a = Vec2f( m_joint->m_Position[0] + a[0] * radius , m_joint->m_Position[1] + a[1] * radius );
	b = Vec2f( m_joint->m_Position[0] + b[0] * radius, m_joint->m_Position[1] + b[1] * radius );
	
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( a[0],a[1]);
  glVertex2f( b[0], b[1]);
  glEnd();
}

// return the c: C(x1, y1, xj, yj, x2, y2) = arccos( a dot b / |a|*|b|) - angle
double AngularConstraint::getC(){
	double* a = new double[2];
	a[0] = m_p1->m_Position[0] - m_joint->m_Position[0];
	a[1] = m_p1->m_Position[1] - m_joint->m_Position[1];
	double* b = new double[2];
	b[0] = m_p2->m_Position[0] - m_joint->m_Position[0];
	b[1] = m_p2->m_Position[1] - m_joint->m_Position[1];
	//return 0;
	return acos(vecDot(2, a, b) /( sqrt(a[0] * a[0] + a[1] * a[1])*sqrt(b[0] * b[0] + b[1] * b[1]))) - m_angle;
}

// return the cdot: Cdot(x, y) = |v_r|sin/|r| met sin=sqrt(1-cos^2) en cos = dot /(|v_r||r|) , r/in{a,b}
double AngularConstraint::getCdot(){

	return 0;
}

//return J, if there are more use same order as particl
//https://www.wolframalpha.com/input/?i=dif++arccos%28%28x*c%2Bb*d%29%2F%28sqrt%28x%5E2+%2B+b%5E2%29*sqrt%28c%5E2+%2B+d%5E2%29%29%29%29
//https://www.wolframalpha.com/input/?i=dif++arccos%28%28%28a-x%29*%28d-f%29%2B%28b-x%29*%28g-f%29%29%2F%28sqrt%28%28a-x%29%5E2+%2B+%28b-x%29%5E2%29*sqrt%28%28d-f%29%5E2+%2B+%28g-f%29%5E2%29%29%29%29
//https://www.wolframalpha.com/input/?i=D%5BArcCos%5B%28%28b+-+h%29+%28g+-+h%29+%2B+%28a+-+c%29+%28d+-+c%29%29%2F%28Sqrt%5B%28b+-+h%29%5E2+%2B+%28a+-+c%29%5E2%5D+Sqrt%5B%28g+-+h%29%5E2+%2B+%28d+-+c%29%5E2%5D%29%5D%2C+h%5D
vector<Vec2f> AngularConstraint::getJ(){
	vector<Vec2f> result;
	float a = m_p1->m_Position[0];
	float b = m_p1->m_Position[1];
	float c = m_joint->m_Position[0];
	float d = m_p2->m_Position[0];
	float g = m_p2->m_Position[1];
	float h = m_joint->m_Position[1];
	float ans = acos(((b - h) * (g - h) + (a - c) * (d - c)) / (sqrt((b - h) * (b - h) + (a - c) * (a - c))* sqrt((g - h)* (g - h) + (d - c) * (d - c))));
	result.push_back(Vec2f(
		((b - h) * (b * (c - d) - c * g + a * (g - h) + d * h)) / pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * sqrt((c - d) * (c - d) + (g - h) * (g - h)) * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h) * (b * c - b * d + a * g - c * g - a * h + d * h) )/ (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h))))
		,
		((a - c) * (b * (-c + d) + c * g - d * h + a * (-g + h))) / pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * sqrt((c - d) * (c - d) + (g - h) * (g - h)) * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h) *(b * c - b * d + a * g - c * g - a * h + d * h)) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))))
		);
	if (!isnormal(result[result.size() - 1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	//printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	result.push_back(Vec2f(
		((g - h) * (b * (-c + d) + c * g - d * h + a * (-g + h))) / (sqrt((a - c) * (a - c) + (b - h) * (b - h)) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2) * sqrt((b * c - b * d + a * g - c * g - a * h + d * h) *(b * c - b * d + a * g - c * g - a * h + d * h) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))))
		,
		-(((c - d) * (b * (c - d) - c * g + a * (g - h) + d * h)) / (sqrt((a - c) * (a - c) + (b - h) * (b - h)) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2 * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h)*(b * c - b * d + a * g - c * g - a * h + d * h)) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h))))))
		)));
	if (!isnormal(result[result.size() - 1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	//printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	result.push_back(Vec2f(
		-(((-c + d) * ((a - c) * (a - c) + (b - h) * (b - h)) * ((a - c) * (-c + d) + (b - h) * (g - h)) + (-a + 2 * c - d) * ((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) + (a - c) * ((a - c) * (-c + d) + (b - h) * (g - h)) * ((c - d) * (c - d) + (g - h) * (g - h))) / (sqrt(1 - (((a - c) * (-c + d) + (b - h) * (g - h)) * ((a - c) * (-c + d) + (b - h) * (g - h))) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))) * pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2)))
		,
		-((((a - c) * (-c + d) + (b - h) * (g - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) * (b - h) + ((a - c) * (a - c) + (b - h) * (b - h)) * ((a - c) * (-c + d) + (b - h) * (g - h)) * (g - h) + ((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) * (-b - g + 2 * h)) / (sqrt(1 - (((a - c) * (-c + d) + (b - h) * (g - h)) * ((a - c) * (-c + d) + (b - h) * (g - h))) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))) * pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2)))
		));
	if (!isnormal(result[result.size() - 1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	//printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	//printf("\n\n");
	return result;
}

//return Jdot, if there are more use same order as particle
vector<Vec2f> AngularConstraint::getJdot(){
	/*double* a = new double[2];
	a[0] = m_p1->m_Velocity[0] - m_joint->m_Velocity[0];
	a[1] = m_p1->m_Velocity[1] - m_joint->m_Velocity[1];
	double* b = new double[2];
	b[0] = m_p2->m_Velocity[0] - m_joint->m_Velocity[0];
	b[1] = m_p2->m_Velocity[1] - m_joint->m_Velocity[1];
	vector<Vec2f> result;
	result.push_back(Vec2f(
		(a[1] * (b[1] * a[0] - a[1] * b[0])) /
		(pow(a[1] * a[1] + a[0] * a[0], 3 / 2)*sqrt(b[0] * b[0] + b[1] * b[1])*
		sqrt(((a[1] * b[0] - b[1] * a[0])*(a[1] * b[0] - b[1] * a[0])) / (a[1] * a[1] + a[0] * a[0])*(b[0] * b[0] + b[1] * b[1])))
		,
		(a[0] * (b[1] * a[1] - a[0] * b[1])) /
		(pow(a[0] * a[0] + a[1] * a[1], 3 / 2)*sqrt(b[0] * b[0] + b[1] * b[1])*
		sqrt(((a[0] * b[1] - b[0] * a[1])*(a[0] * b[1] - b[0] * a[1])) / (a[0] * a[0] + a[1] * a[1])*(b[0] * b[0] + b[1] * b[1])))));
	result.push_back(Vec2f(
		(b[1] * (a[1] * b[0] - a[0] * b[1])) /
		(pow(b[1] * b[1] + b[0] * b[0], 3 / 2)*sqrt(a[0] * a[0] + a[1] * a[1])*
		sqrt(((a[0] * b[1] - b[0] * a[1])*(a[0] * b[1] - b[0] * a[1])) / (a[0] * a[0] + a[1] * a[1])*(b[1] * b[1] + b[0] * b[0])))
		,
		(b[0] * (a[0] * b[1] - a[1] * b[0])) /
		(pow(b[0] * b[0] + b[1] * b[1], 3 / 2)*sqrt(a[0] * a[0] + a[1] * a[1])*
		sqrt(((a[1] * b[0] - a[0] * b[1])*(a[1] * b[0] - a[0] * b[1])) / (a[0] * a[0] + a[1] * a[1])*(b[0] * b[0] + b[1] * b[1])))));
	*/
	vector<Vec2f> result;
	float a = m_p1->m_Velocity[0];
	float b = m_p1->m_Velocity[1];
	float c = m_joint->m_Velocity[0];
	float d = m_p2->m_Velocity[0];
	float g = m_p2->m_Velocity[1];
	float h = m_joint->m_Velocity[1];	

	result.push_back(Vec2f(
		((b - h) * (b * (c - d) - c * g + a * (g - h) + d * h)) / pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * sqrt((c - d) * (c - d) + (g - h) * (g - h)) * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h) * (b * c - b * d + a * g - c * g - a * h + d * h)) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h))))
		,
		((a - c) * (b * (-c + d) + c * g - d * h + a * (-g + h))) / pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * sqrt((c - d) * (c - d) + (g - h) * (g - h)) * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h) *(b * c - b * d + a * g - c * g - a * h + d * h)) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))))
		);
	if (!isnormal(result[result.size()-1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	result.push_back(Vec2f(
		((g - h) * (b * (-c + d) + c * g - d * h + a * (-g + h))) / (sqrt((a - c) * (a - c) + (b - h) * (b - h)) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2) * sqrt((b * c - b * d + a * g - c * g - a * h + d * h) *(b * c - b * d + a * g - c * g - a * h + d * h) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))))
		,
		-(((c - d) * (b * (c - d) - c * g + a * (g - h) + d * h)) / (sqrt((a - c) * (a - c) + (b - h) * (b - h)) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2 * sqrt(((b * c - b * d + a * g - c * g - a * h + d * h)*(b * c - b * d + a * g - c * g - a * h + d * h)) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h))))))
		)));
	if (!isnormal(result[result.size() - 1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	result.push_back(Vec2f(
		-(((-c + d) * ((a - c) * (a - c) + (b - h) * (b - h)) * ((a - c) * (-c + d) + (b - h) * (g - h)) + (-a + 2 * c - d) * ((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) + (a - c) * ((a - c) * (-c + d) + (b - h) * (g - h)) * ((c - d) * (c - d) + (g - h) * (g - h))) / (sqrt(1 - (((a - c) * (-c + d) + (b - h) * (g - h)) * ((a - c) * (-c + d) + (b - h) * (g - h))) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))) * pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2)))
		,
		-((((a - c) * (-c + d) + (b - h) * (g - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) * (b - h) + ((a - c) * (a - c) + (b - h) * (b - h)) * ((a - c) * (-c + d) + (b - h) * (g - h)) * (g - h) + ((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)) * (-b - g + 2 * h)) / (sqrt(1 - (((a - c) * (-c + d) + (b - h) * (g - h)) * ((a - c) * (-c + d) + (b - h) * (g - h))) / (((a - c) * (a - c) + (b - h) * (b - h)) * ((c - d) * (c - d) + (g - h) * (g - h)))) * pow(((a - c) * (a - c) + (b - h) * (b - h)), 3 / 2) * pow(((c - d) * (c - d) + (g - h) * (g - h)), 3 / 2)))
		));
	if (!isnormal(result[result.size() - 1][0])){
		result[result.size() - 1][0] = 0;
	}
	if (!isnormal(result[result.size() - 1][1])){
		result[result.size() - 1][1] = 0;
	}
	printf("%f \t %f\n", result[result.size() - 1][0], result[result.size() - 1][1]);
	printf("\n\n\n");
	return result;

}
vector<Particle> AngularConstraint::getParticles(){
	vector<Particle> result;
	result.push_back(*m_p1);
	result.push_back(*m_p2);
	result.push_back(*m_joint);
	return result;
}