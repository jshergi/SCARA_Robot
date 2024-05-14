#ifndef MATRIXOPERATIONS_H
#define MATRIXOPERATIONS_H

#include <iostream>
#include <cmath>
#include "ensc-488.h"
#include <math.h>

using namespace std;

// MVSIZE represents Matrix & Vector Size 
const int MVSIZE = 4;
const double EPSILON = 1e-7;

const double L1 = 405.0;
const double L2 = 70.0;
const double L3 = 195.0;
const double L4 = 142.0;
const double L5 = 410.0;
const double L6 = 80.0;
const double L7 = 60.0;

const double j1Max = 150.001;
const double j1Min = -150.001;
const double j2Max = 100.001;
const double j2Min = -100.001;
const double j4Max = 160.001;
const double j4Min = -160.001;

const double d3Max = -100.0;
const double d3Min = -200.0;

const double vMaxR = 150.0;
const double vMinR = -150.0;
const double vMaxP = 50.0;
const double vMinP = -50.0;

const double aMaxR = 600.0;
const double aMinR = -600.0;
const double aMaxP = 200.0; 
const double aMinP = -200;


typedef double vec[MVSIZE];
typedef double matrix[MVSIZE][MVSIZE];
typedef double pva[MVSIZE * 3 + 1];
typedef bool flag[MVSIZE];

const matrix TSB = { {1, 0, 0, 0},
					{0, 1, 0, 0},
					{0, 0, 1, 0},
					{0, 0, 0, 1} };

const matrix TWT = { {1, 0, 0, 0},
					{0, 1, 0, 0},
					{0, 0, 1, L7},
					{0, 0, 0, 1} };

// UTOI function: part 2:2
inline void UTOI(vec uform, matrix& iform) {
	uform[3] = DEG2RAD(uform[3]);

	iform[0][0] = cos(uform[3]);
	iform[0][1] = -sin(uform[3]);
	iform[0][2] = 0.0;
	iform[0][3] = uform[0];

	iform[1][0] = sin(uform[3]);
	iform[1][1] = cos(uform[3]);
	iform[1][2] = 0.0;
	iform[1][3] = uform[1];

	iform[2][0] = 0.0;
	iform[2][1] = 0.0;
	iform[2][2] = -1.0; // Or -1.0 need to confirm
	iform[2][3] = uform[2];

	iform[3][0] = 0.0;
	iform[3][1] = 0.0;
	iform[3][2] = 0.0;
	iform[3][3] = 1.0;
}

//ITOU function: part 2:2
//Textbook convention
inline void ITOU(const matrix iform, vec& uform) {
	uform[0] = iform[0][3];
	uform[1] = iform[1][3];
	uform[2] = iform[2][3];
	uform[3] = atan2(iform[1][0], iform[0][0]);
	uform[3] = RAD2DEG(uform[3]);
}

// Description: Prints Matrix
inline void printMatrix(const matrix& mat) {
	for (int i = 0; i < MVSIZE; i++) {
		for (int j = 0; j < MVSIZE; j++) {
			std::cout << mat[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

// Description: Multiplies 2 matrices
// https://www.geeksforgeeks.org/c-program-multiply-two-matrices/
inline void TMULT(const matrix brela, const matrix crelb, matrix& crela) {
	for (int i = 0; i < MVSIZE; i++) {
		for (int j = 0; j < MVSIZE; j++) {
			crela[i][j] = 0;
			for (int k = 0; k < MVSIZE; k++) {
				crela[i][j] += (brela[i][k] * crelb[k][j]);
			}
			if (fabs(crela[i][j]) < EPSILON) {
				crela[i][j] = 0.0;
			}
		}
	}
}

// Description: Invert Matrix
inline void TINVERT(const matrix brela, matrix& arelb) {
	double a = -(brela[0][0] * brela[0][3] + brela[1][0] * brela[1][3] + brela[2][0] * brela[2][3]);
	double b = -(brela[0][1] * brela[0][3] + brela[1][1] * brela[1][3] + brela[2][1] * brela[2][3]);
	double c = -(brela[0][2] * brela[0][3] + brela[1][2] * brela[1][3] + brela[2][2] * brela[2][3]);

	// Transpose
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			arelb[i][j] = brela[j][i];
		}
	}

	arelb[0][3] = a;
	arelb[1][3] = b;
	arelb[2][3] = c;

	arelb[3][0] = 0;
	arelb[3][1] = 0;
	arelb[3][2] = 0;
	arelb[3][3] = 1;

}

inline bool j1LimitExc(double angle) {
	return ((angle > j1Max) || (angle < j1Min));
}

inline bool j2LimitExc(double angle) {
	return ((angle > j2Max) || (angle < j2Min));
}

inline bool j4LimitExc(double angle) {
	return ((angle > j4Max) || (angle < j4Min));
}

inline bool d3LimitExc(double distance) {
	return ((distance > d3Max) || (distance < d3Min));
}

inline void KIN(double theta1, double theta2, double d3, double theta4, matrix& output) {
	theta1 = DEG2RAD(theta1);
	theta2 = DEG2RAD(theta2);
	theta4 = DEG2RAD(theta4);
	matrix T01 = {
		{cos(theta1), -sin(theta1), 0, 0},
		{sin(theta1), cos(theta1), 0, 0},
		{0, 0, 1, L1},
		{0, 0, 0, 1} };

	matrix T12 = {
		{cos(theta2), -sin(theta2), 0, L3},
		{sin(theta2), cos(theta2), 0, 0},
		{0, 0, 1, L2},
		{0, 0, 0, 1} };

	matrix T23 = {
		{1, 0, 0, L4},
		{0, -1, 0, 0},
		{0, 0, -1, -d3},
		{0, 0, 0, 1} };

	matrix T34 = {
		{cos(theta4), -sin(theta4), 0, 0},
		{sin(theta4), cos(theta4), 0, 0},
		{0, 0, 1, (L5 + L6)},
		{0, 0, 0, 1} };

	matrix T02;
	matrix T03;
	TMULT(T01, T12, T02);
	TMULT(T02, T23, T03);
	TMULT(T03, T34, output);
	//printMatrix(output);
}

inline void WHERE(double theta1, double theta2, double d3, double theta4, vec& output) {
	if (!j1LimitExc(theta1) && !j2LimitExc(theta2) && !j4LimitExc(theta4) && !d3LimitExc(d3)) {
		matrix T04;
		//cout << "IN HERE: \n"; 
		KIN(theta1, theta2, d3, theta4, T04);
		matrix TSW;
		matrix TST;
		TMULT(TSB, T04, TSW);
		TMULT(TSW, TWT, TST);
		ITOU(TST, output);
	}
	else {
		cout << "\n INPUT PARAMETERS INVALID!\n";
		for (int i = 0; i < MVSIZE; i++) {
			output[i] = 0.0;
		}
	}
}

inline bool inReach(double x, double y, double z) {
	if ((sqrt(pow(x, 2) + pow(y, 2)) > (L4 + L3)) || (sqrt(pow(x, 2) + pow(y, 2)) < (L3 - L4))) {
		//cout << "(" << x << ", " << y << ") -> " << sqrt(pow(x, 2) + pow(y, 2)) << " vs " << L4 + L3 << endl;
		cout << "ERROR: Not in the reachable workspace!";
		return false;
	}
	if (z > 125 || z < 25) {
		cout << "ERROR: Not in the reachable workpace!\n";
		//cout << z << x << y << endl;
		return false;
	}

	return true;
}

inline double revolveAngle(double angle, double maxAngle) {
	double angleToReturn = angle; 
	if (angle > maxAngle) {
		angleToReturn -= 360;
	}
	
	if (angle < -maxAngle) {
		angleToReturn += 360;
	}
	return angleToReturn; 

	/*if (angle > maxangle) {
		angle -= 360;
	}

	if (angle < -maxangle) {
		angle += 360;
	}

	if (angle == -0) {
		angle = 0;
	}*/
}

//Input the positive and negative joint values for maxANgle
inline double fixBorderCase(double angle, double maxAngle) {

	//Positive case
	if (angle > 0) {

		if (angle > maxAngle && angle < maxAngle + 0.001) {
			return maxAngle;
		}
		else {
			return angle;
		}
	}
	//Negative case
	else {

		if (angle < maxAngle * (-1) && angle > maxAngle * (-1) - 0.001) {
			return -maxAngle;
		}
		else {
			return angle;
		}
	}

}

inline void INVKIN(matrix TBW, vec current, vec& near, vec& far, bool& sol) {
	vec position = { 0, 0, 0, 0 };
	sol = false;
	ITOU(TBW, position);

	double x = position[0];
	double y = position[1];
	double z = position[2];
	double phi = position[3];
	//cout << "z " << z << "phi" << phi << endl;

	// Check workspace limitations
	bool valid = inReach(x, y, z);
	if (!valid) {
		return; // return for now
	}

	double c2;
	double s2Pos;
	double s2Neg;
	double theta1Pos = 0.0f;
	double theta1Neg = 0.0f;
	double theta2Pos = 0.0f;
	double theta2Neg = 0.0f;
	double theta41 = 0.0f;
	double theta42 = 0.0f;
	double d3 = 0.0f;

	// 2 Number of Solutions

	c2 = (pow(x, 2) + pow(y, 2) - pow(L3, 2) - pow(L4, 2)) / (2 * L3 * L4);
	s2Pos = sqrt(1 - pow(c2, 2));
	s2Neg = -sqrt(1 - pow(c2, 2));

	theta2Pos = atan2(s2Pos, c2);
	//theta2Pos = RAD2DEG(theta2Pos); 
	theta2Neg = atan2(s2Neg, c2);
	//theta2Neg = RAD2DEG(theta2Neg);

	theta1Pos = atan2(((L3 + L4 * cos(theta2Pos)) * y - L4 * sin(theta2Pos) * x), ((L3 + L4 * cos(theta2Pos)) * x + L4 * sin(theta2Pos) * y));
	theta1Neg = atan2(((L3 + L4 * cos(theta2Neg)) * y - L4 * sin(theta2Neg) * x), ((L3 + L4 * cos(theta2Neg)) * x + L4 * sin(theta2Neg) * y));

	theta1Pos = RAD2DEG(theta1Pos);
	//cout << "\n" << theta1Pos; 
	theta1Neg = RAD2DEG(theta1Neg);
	//cout << "\n" << theta1Neg;
	//added after comment
	theta2Neg = RAD2DEG(theta2Neg);
	//cout << "\n" << theta2Neg;
	theta2Pos = RAD2DEG(theta2Pos);
	//cout << "\n" << theta2Pos;

	
	

	// CALCULATE d3
	d3 = L1 + L2 - L5 - L6 - z - L7;


	// CALCULATE theta41 AND theta42
	// (phi is negative because 
	
	//theta41 = -phi + theta1Pos + theta2Pos;
	theta41 = -phi + theta1Pos + theta2Pos;
	//cout << "theta4: " << theta41;
	//theta42 = -phi + theta1Neg + theta2Neg;
	theta42 = -phi + theta1Neg + theta2Neg;
	//cout << "\ntheta42: " << theta42;

	// Revolve angles to be within [-359.99 359.99]
	theta1Pos = revolveAngle(theta1Pos, j1Max);
	theta1Neg = revolveAngle(theta1Neg, j1Max);
	theta2Pos = revolveAngle(theta2Pos, j2Max);
	theta2Neg = revolveAngle(theta2Neg, j2Max);
	theta41 = revolveAngle(theta41, j4Max); 
	theta42 = revolveAngle(theta42, j4Max);

	//Fix angles so that if they are between 150.000 150.001 just set em to 150
	//J1: 150 J2: 100 J3: 160

	theta1Pos = fixBorderCase(theta1Pos, 150);
	theta1Neg = fixBorderCase(theta1Neg, 150);
	theta2Pos = fixBorderCase(theta2Pos, 100);
	theta2Neg = fixBorderCase(theta2Neg, 100);
	theta41 = fixBorderCase(theta41, 160);
	theta42 = fixBorderCase(theta42, 160);

	bool test = true;

	//cout << "Solutions: " << endl;
	if (test == true) {
		cout << "\nThe first solution: " << "\nTheta 1: " << theta1Pos << "\nTheta 2: " << theta2Pos << "\nd3     : " << d3 << "\nTheta 4: " << theta41;
		cout << "\n\nThe second solution: " << "\nTheta 1: " << theta1Neg << "\nTheta 2: " << theta2Neg << "\nd3     : " << d3 << "\nTheta 4: " << theta42;
		cout << endl << endl;
	}
	

	bool first = false;
	bool second = false;

	//cout << !j1LimitExc(theta1Pos) << !j2LimitExc(theta2Pos) << !d3LimitExc(d3) << !j4LimitExc(theta41); 

	// CHECK VALIDITY OF SOLUTIONS
	if (!j1LimitExc(theta1Pos) && !j2LimitExc(theta2Pos) && !d3LimitExc(d3) && !j4LimitExc(theta41)) {
	//	cout << "  The first solution is valid." << endl;
		first = true;
		sol = true;
	}

	if (!j1LimitExc(theta1Neg) && !j2LimitExc(theta2Neg) && !d3LimitExc(d3) && !j4LimitExc(theta42)) {
		//cout << "  The second solution is valid." << endl;
		second = true;
		sol = true;
	}





	/*if (!j1LimitExc(theta1Neg) && !j1LimitExc(theta1Pos)
		&& !j2LimitExc(theta2Neg) && !j2LimitExc(theta2Pos)
		&& !j4LimitExc(theta41) && !j4LimitExc(theta42)
		&& !d3LimitExc(d3)) {

		sol = true;
	}*/

	vec solutions[2] = { {0,0,0,0},
						{0,0,0,0} };

	solutions[0][0] = theta1Pos;
	solutions[0][1] = theta2Pos;
	solutions[0][2] = d3;
	solutions[0][3] = theta41;

	//cout << "\nSolutions[0]" << endl;
	for (int i = 0; i < 4; i++) {
		//cout << solutions[0][i] << " ";
	}

	//cout << "\nSolutions[1]" << endl;
	solutions[1][0] = theta1Neg;
	solutions[1][1] = theta2Neg;
	solutions[1][2] = d3;
	solutions[1][3] = theta42;

	for (int i = 0; i < 4; i++) {
		//cout << solutions[1][i] << " ";
	}
	
	double solution0 = sqrt(pow((current[0] - solutions[0][0]), 2) + pow((current[1] - solutions[0][1]), 2) + pow((current[3] - solutions[0][3]), 2));
	double solution1 = sqrt(pow((current[0] - solutions[1][0]), 2) + pow((current[1] - solutions[1][1]), 2) + pow((current[3] - solutions[1][3]), 2));

	if (sol == true && first == true && second == true) {
		if (solution0 > solution1) {
			//cout << abs(solutions[0] - current) << abs(solutions[1] - current);
			for (int i = 0; i < 4; i++) {
				far[i] = solutions[0][i];
				near[i] = solutions[1][i];
			}
		}
		else {
			for (int i = 0; i < 4; i++) {
				far[i] = solutions[1][i];
				near[i] = solutions[0][i];
			}
		}
	}

	else if (sol == true && first == true && second == false) {
		cout << "Only one solution. " << endl;
		for (int i = 0; i < 4; i++) {
			//far[i] = solutions[0][i];
			near[i] = solutions[0][i];
			far[i] = 0;
		}

	}

	else if (sol == true && first == false && second == true) {
		cout << "Only one solution. " << endl;
		for (int i = 0; i < 4; i++) {
			//far[i] = solutions[0][i];
			near[i] = solutions[1][i];
			far[i] = 0;
		}

	}

	else if  (sol == false) {
		cout << "ERROR: values exceeding limits!";
	}

	return;

}

inline void SOLVE(matrix TST, vec current, vec& near, vec& far, bool& sol) {
	matrix TBS;
	matrix TTW;
	matrix TBT;
	matrix TBW;

	// From textbook 
	// TBW = (TBS)(TST)(TWT)^-1

	TINVERT(TSB, TBS);
	TINVERT(TWT, TTW);

	TMULT(TBS, TST, TBT);
	//TMULT(TBT, TTW, TBW);
	//account for offset from wrist to tool (60mm)
	//TBW[2][3] = TBW[2][3] + 60;
	INVKIN(TBT, current, near, far, sol);


}
// Select appropriate inverse kinematics solution for each via point
inline void viaInvKin(vec point, vec& sol, bool& solExist) {
	JOINT current; 
	GetConfiguration(current);
	matrix a; 
	vec near = { 0,0,0,0 };
	vec far;
	bool solution;

	UTOI(point, a);
	SOLVE(a, current, near, far, solution);

	if (solution == true) {
		for (int i = 0; i < MVSIZE; i++) {
			sol[i] = near[i]; 
		}
	}
	else {
		for (int i = 0; i < MVSIZE; i++) {
			sol[i] = 0;
		}
	}
	solExist = solution; 

}

// Used to calculate the velocity heuristic 
//inline void computeVelH(vec via1, vec via2, vec via3, vec via4, vec& v1Via, vec& v2Via, vec& v3Via, vec& v4Via, double time) {
//	
//	vec theta1 = { via1[0], via2[0], via3[0], via4[0] };
//	vec theta2 = { via1[1], via2[1], via3[1], via4[1] };
//	vec d3 = { via1[2], via2[2], via3[2], via4[2] };
//	vec theta4 = { via1[3], via2[3], via3[3], via4[3] };
//
//	for (int i = 0; i < MVSIZE; i++) {
//		v1Via[i] = 0;
//		v2Via[i] = 0;
//		v3Via[i] = 0;
//		v4Via[i] = 0;
//	}
//
//	for (int i = 1; i < MVSIZE; i++) {
//		v1Via[i - 1] = (((theta1[i] - theta1[i - 1]) / time) + ((theta1[i + 1] - theta1[i]) / time)) / 2;
//	}
//	for (int i = 1; i < MVSIZE; i++) {
//		v2Via[i - 1] = (((theta2[i] - theta2[i - 1]) / time) + ((theta2[i + 1] - theta2[i]) / time)) / 2;
//	}
//	for (int i = 1; i < MVSIZE; i++) {
//		v3Via[i - 1] = (((d3[i] - d3[i - 1]) / time) + ((d3[i + 1] - d3[i]) / time)) / 2;
//	}
//	for (int i = 1; i < MVSIZE; i++) {
//		v4Via[i - 1] = (((theta4[i] - theta4[i - 1]) / time) + ((theta4[i + 1] - theta4[i]) / time)) / 2;
//	}
//
//	
//	
//}

// Function definition from textbook (Part 7:1) 
// Computing coefficient matrix
inline void CUBCOEF(vec p0, vec pf, vec v0, double t, matrix& CC, bool last) {
	vec aj1; 
	vec aj2; 
	vec aj3; 
	vec aj4; 

	double p0t1, pft1, vft1;
	double p0t2, pft2, vft2;
	double p0d3, pfd3, vfd3;
	double p0t4, pft4, vft4;

	p0t1 = p0[0];
	pft1 = pf[0]; 

	p0t2 = p0[1]; 
	pft2 = pf[1];

	p0d3 = p0[2];
	pfd3 = pf[2];

	p0t4 = p0[3]; 
	pft4 = pf[3];

	//v0t1 = v0[0];
	//v0t2 = v0[1];
	//v0d3 = v0[2]; 
	//v0t4 = v0[3];

	// Option 2 Heuristic (from text obtained simple heuristic =  slope of line) 
	vft1 = (pft1 - p0t1) / t; 
	vft2 = (pft2 - p0t2) / t; 
	vfd3 = (pfd3 - p0d3) / t; 
	vft4 = (pft4 - p0t4) / t; 

	// First Method from textbook, if stopping at via points, cubic spline
	if (last) {
		vft1 = 0;
		vft2 = 0;
		vfd3 = 0;
		vft4 = 0;
	}

	// Solving for a0, a1, a2, a3 for all joints 

	// Theta 1
	aj1[0] = p0t1;
	aj1[1] = v0[0];
	aj1[2] = (3 / pow(t, 2)) * (pft1 - p0t1) - ((2 * v0[0]) / t) - (vft1 / t);
	aj1[3] = (-2 / pow(t, 3)) * (pft1 - p0t1) + ((vft1 + v0[0]) / pow(t, 2));

	// Theta 2
	aj2[0] = p0t2;
	aj2[1] = v0[1];
	aj2[2]= (3 / pow(t, 2)) * (pft2 - p0t2) - ((2 * v0[1]) / t) - (vft2 / t);
	aj2[3] = (-2 / pow(t, 3)) * (pft2 - p0t2) + ((vft2 + v0[1]) / pow(t, 2));

	// d3 
	aj3[0] = p0d3;
	aj3[1] = v0[2];
	aj3[2] = (3 / pow(t, 2)) * (pfd3 - p0d3) - ((2 * v0[2]) / t) - (vfd3 / t);
	aj3[3] = (-2 / pow(t, 3)) * (pfd3 - p0d3) + ((vfd3 + v0[2]) / pow(t, 2));

	// Theta 4
	aj4[0] = p0t4;
	aj4[1] = v0[3];
	aj4[2] = (3 / pow(t, 2)) * (pft4 - p0t4) - ((2 * v0[3]) / t) - (vft4 / t);
	aj4[3] = (-2 / pow(t, 3)) * (pft4 - p0t4) + ((vft4 + v0[3]) / pow(t, 2));

	// Coefficient matrix
	matrix CC1 = {
		{aj1[0], aj1[1], aj1[2], aj1[3]},
		{aj2[0], aj2[1], aj2[2], aj2[3]},
		{aj3[0], aj3[1], aj3[2], aj3[3]},
		{aj4[0], aj4[1], aj4[2], aj4[3]} };

	for (int i = 0; i < MVSIZE; i++) {
		for (int j = 0; j < MVSIZE; j++) {
			CC[i][j] = CC1[i][j]; 
		}
	}
	

}

//inline bool velRCheck(double vR)
//{
//	if (vR > vMaxR || vR < vMinR) {
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//inline bool velPCheck(double vR)
//{
//	if (vR > vMaxP || vR < vMinP) {
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//inline bool accCheck(double a)
//{
//	if (a > aMaxR || a < aMaxR) {
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}




inline void velAccCheck(bool& fail, double v1, double v2, double v3, double v4, double a1, double a2, double a3, double a4) {
	fail = false; 
	if (v1 > vMaxR || v1 < vMinR) {
		cout << "Violation: Joint 1 velocity." << endl; 
		fail = true; 
	}
	if (v2 > vMaxR || v2 < vMinR) {
		cout << "Violation: Joint 2 velocity." << endl;
		fail = true;
	}
	if (v3 > vMaxP || v3 < vMinP) {
		cout << "Violation: Joint 3 velocity." << endl;
		fail = true;
	}
	if (v4 > vMaxR || v4 < vMinR) {
		cout << "Violation: Joint 4 velocity." << endl;
		fail = true;
	}
	if (a1 > aMaxR || a1 < aMinR) {
		cout << "Violation: Joint 1 acceleration." << endl;
		fail = true;
	}
	if (a2 > aMaxR || a2 < aMinR) {
		cout << "Violation: Joint 2 acceleration." << endl;
		fail = true;
	}
	if (a3 > aMaxP || a3 < aMinP) {
		cout << "Violation: Joint 3 acceleration." << endl;
		fail = true;
	}
	if (a4 > aMaxR || a4 < aMinR) {
		cout << "Violation: Joint 4 acceleration." << endl;
		fail = true;
	}
}

inline void trajectoryPlanner(matrix CC, double t, pva& solutions) {
	vec aj1;
	vec aj2; 
	vec aj3; 
	vec aj4; 

	double p1, p2, p3, p4;
	double v1, v2, v3, v4; 
	double a1, a2, a3, a4; 

	for (int i = 0; i < MVSIZE; i++) {
		aj1[i] = CC[0][i];
	}
	p1 = aj1[0] + aj1[1] * t + aj1[2] * pow(t, 2) + aj1[3] * pow(t, 3); 
	v1 = aj1[1] + 2 * aj1[2] * t + 3 * aj1[3] * pow(t, 2); 
	a1 = 2 * aj1[2] + 6 * aj1[3] * t;

	for (int i = 0; i < MVSIZE; i++) {
		aj2[i] = CC[1][i]; 
	}
	p2 = aj2[0] + aj2[1] * t + aj2[2] * pow(t, 2) + aj2[3] * pow(t, 3);
	v2 = aj2[1] + 2 * aj2[2] * t + 3 * aj2[3] * pow(t, 2);
	a2 = 2 * aj2[2] + 6 * aj2[3] * t;

	for (int i = 0; i < MVSIZE; i++) {
		aj3[i] = CC[2][i];
	}
	p3 = aj3[0] + aj3[1] * t + aj3[2] * pow(t, 2) + aj3[3] * pow(t, 3);
	v3 = aj3[1] + 2 * aj3[2] * t + 3 * aj3[3] * pow(t, 2);
	a3 = 2 * aj3[2] + 6 * aj3[3] * t;

	for (int i = 0; i < MVSIZE; i++) {
		aj4[i] = CC[3][i];
	}
	p4 = aj4[0] + aj4[1] * t + aj4[2] * pow(t, 2) + aj4[3] * pow(t, 3);
	v4 = aj4[1] + 2 * aj4[2] * t + 3 * aj4[3] * pow(t, 2);
	a4 = 2 * aj4[2] + 6 * aj4[3] * t;

	bool fail; 
	velAccCheck(fail, v1, v2, v3, v4, a1, a2, a3, a4); 
	
	solutions[0] = p1;
	solutions[1] = p2;
	solutions[2] = p3; 
	solutions[3] = p4;

	solutions[4] = v1;
	solutions[5] = v2;
	solutions[6] = v3;
	solutions[7] = v4; 

	solutions[8] = a1;
	solutions[9] = a2;
	solutions[10] = a3; 
	solutions[11] = a4; 

	solutions[12] = fail; 
	//cout << "Printing Solutions[12]" << solutions[12] << endl;



}

#endif
