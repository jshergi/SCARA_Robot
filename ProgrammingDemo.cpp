// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include "matrixOperations.h"
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <fstream>

using namespace std;


// Time Reference
//https://stackoverflow.com/questions/4184468/sleep-for-milliseconds
void sleepSeconds(int milliSeconds) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milliSeconds));
}

void printL() {
	JOINT currentJoint;
	vec out; 
	GetConfiguration(currentJoint);
	WHERE(currentJoint[0], currentJoint[1], currentJoint[2], currentJoint[3], out);
	cout << "Goal Position after Moving Robot (Forward Kinematics): " << endl;
	cout << "\n X: " << out[0];
	cout << "\n Y: " << out[1];
	cout << "\n Z: " << out[2];
	cout << "\n Phi: " << out[3] << endl << endl;
	if (out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0) {
		cout << "\nDon't try this configuration on the physical robot! Rounding error has caused a joint violation!" << endl;
	}
	
}

// Plotting Reference
//https://stackoverflow.com/questions/25201131/writing-csv-files-from-c
void plotter(double partT, matrix CC, double prevTime, bool& flag, bool& velFlag, std::ofstream& p, std::ofstream& v, std::ofstream& accel, std::ofstream& vias) {
	double time = 0; 
	double timeStep = partT / 10;
	vec output = { 0,0,0,0 };
	pva solutions;  
	double x; 
	double y; 
	velFlag = false;
	bool flagVel = false; 
	flag = false;

	while (true) {
		trajectoryPlanner(CC, time, solutions);
		vec position = { solutions[0], solutions[1], solutions[2], solutions[3] };
		vec vel = { solutions[4], solutions[5], solutions[6], solutions[7] };
		vec acc = { solutions[8], solutions[9], solutions[10], solutions[11] };
		flagVel = solutions[12];
		if (flagVel) {
			velFlag = true;
		}
		WHERE(position[0], position[1], position[2], position[3], output);
		x = output[0]; 
		y = output[1]; 
		p << time + prevTime << "," << position[0] << "," << position[1] << "," << position[2] << "," << position[3] << "," << x << "," << y << endl;
		v << time + prevTime << "," << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << endl; 
		accel << time + prevTime << "," << acc[0] << "," << acc[1] << "," << acc[2] << "," << acc[3] << endl;
		time += timeStep; 

		if (output[0] == 0 && output[1] == 0 && output[2] == 0 && output[3] == 0) {
			cout << "Violation: Joint Limit" << endl;
			flag = true; 
		}

		if (time > partT + EPSILON) {
			//cout << "\ntime: " << time << endl;
			break;
			//cout << "done write" << endl; 
		}
	}
	return; 

}

void moveRobot(double partT, matrix CC, double prevTime) {
	double time = 0;
	double timeStep = partT / 10;
	vec output = { 0,0,0,0 };
	pva solutions;
	double x;
	double y;

	while (true) {
		trajectoryPlanner(CC, time, solutions);
		vec position = { solutions[0], solutions[1], solutions[2], solutions[3] };
		vec vel = { solutions[4], solutions[5], solutions[6], solutions[7] };
		vec acc = { solutions[8], solutions[9], solutions[10], solutions[11] };

		//if (time == 0 || time == timeStep *5 || time == timeStep * 10 ) {
		//cout << "Printing Position" << endl; 
		//cout << position[0] << " " << position[1] << " " << position[2] << " " << position[3] << endl; 
		MoveWithConfVelAcc(position, vel, acc);
		// https://www.geeksforgeeks.org/sleep-function-in-cpp/
		sleepSeconds(timeStep * 1000);
		
		//}
		

		time += timeStep;

		if (time > partT) {
			vec vel0 = { 0, 0, 0, 0 };
			MoveWithConfVelAcc(position, vel0, acc);
			break;
			//cout << "done write" << endl; 
		}
	}
	return;
}

int main(int argc, char* argv[])
{
	JOINT q1 = {1000,1000,1000,1000};
	printf("Keep this window in focus, and...\n");

	char ch;
	int c;

	const int ESC = 27;

	//printf("1Press any key to continue \n");
	//printf("2Press ESC to exit \n");

	c = 'a';
	char input = 'o';

	matrix a;
	JOINT current;
	vec near = { 0,0,0,0 };
	vec far;

	vec v0 = { 0,0,0,0 };
	vec via1Inv; 
	vec via2Inv; 
	vec via3Inv; 
	vec goalInv; 

	bool via1S; 
	bool via2S; 
	bool via3S; 
	bool goalS; 

	bool sol;
	
	matrix CC1;
	matrix CC2; 
	matrix CC3; 
	matrix CC4; 
	pva solutions;
	flag flags;

	while (c != ESC) {

		while (input != 'f' && input != 'i' && input != 't') {
			cout << "Do you want to enter joint angles/distances (f), pose of the end effector (i), or plan a trajectory with via points (t)?" << endl;
			input = _getch();
		}
		
		// KINEMATICS (f) -------------------------------------------------------------------------------------------------------------------------
		if (input == 'f') {
			
			while (q1[0] < j1Min || q1[0] > j1Max) {
				cout << "Enter Theta 1 (" << int(j1Min) << " to " << int(j1Max) << "), the angle of the first joint: ";
				cin >> q1[0];
				if (q1[0] < j1Min || q1[0] > j1Max) {
					cout << "\nNot in the vaild range! Please try again." << endl;
				}
			}

			while (q1[1] < j2Min || q1[1] > j2Max) {
				cout << "Enter Theta 2 (" << int(j2Min) << " to " << int(j2Max) << "), the angle of the second joint: ";
				cin >> q1[1];
				if (q1[1] < j2Min || q1[1] > j2Max) {
					cout << "\nNot in the vaild range! Please try again." << endl;
				}
			}

			while (q1[2] < d3Min || q1[2] > d3Max) {
				cout << "Enter Distance 3 (" << int(d3Min) << " to " << int(d3Max) << "), the extension of the third joint: ";
				cin >> q1[2];
				if (q1[2] < d3Min || q1[2] > d3Max) {
					cout << "\nNot in the vaild range! Please try again." << endl;
				}
			}

			while (q1[3] < j4Min || q1[3] > j4Max) {
				cout << "Enter Theta 4 (" << int(j4Min) << " to " << int(j4Max) << "), the angle of the fourth joint: ";
				cin >> q1[3];
				if (q1[3] < j4Min || q1[3] > j4Max) {
					cout << "\nNot in the vaild range! Please try again." << endl;
				}

			}

			vec out;
			WHERE(q1[0], q1[1], q1[2], q1[3], out);
			cout << "Solution (Forward Kinematics): " << endl;
			cout << "\n X: " << out[0];
			cout << "\n Y: " << out[1];
			cout << "\n Z: " << out[2];
			cout << "\n Phi: " << out[3] << endl;

			//cout << sqrt(pow(out[0], 2) + pow(out[1], 2)) << endl << endl;

			MoveToConfiguration(q1);
			//DisplayConfiguration(q1);

			cout << "Grab object? [g] or release object [r], or press any other key to do nothing" << endl;
			input = _getch();

			if (input == 'g') {
				cout << "Closing gripper " << endl;
				Grasp(true);
			}
			else if (input == 'r') {
				cout << "Opening gripper " << endl;
				Grasp(false);
			}
			else {
				//nothing
			}

			//-----
		}

		// INVERSE KINEMATICS (I)
		if (input == 'i') {
			
			cout << "Enter the x value of your desired location: ";
			cin >> q1[0];

			cout << "Enter the y value of your desired location: ";
			cin >> q1[1];

			cout << "Enter the z value of your desired location: ";
			cin >> q1[2];

			cout << "Enter the phi value of your desired location: ";
			cin >> q1[3];

			GetConfiguration(current);

			UTOI(q1, a);

			SOLVE(a, current, near, far, sol);
			// adding this in 
			/*vec solf; 
			viaInvKin(q1, solf, sol); 
			cout << solf[0] << endl; 
			cout << solf[1] << endl; 
			cout << solf[2] << endl; 
			cout << solf[3] << endl; 
			cout << sol << endl; */

			if (sol == true) {
				cout << "\nNearest Solution (Inverse Kinematics):";
				cout << "\n Theta1: " << near[0];
				cout << "\n Theta2: " << near[1];
				cout << "\n Distance3: " << near[2];
				cout << "\n Theta4: " << near[3] << endl;

				if (near[0] != far[0]) {
					cout << "\nFurthest Solution: " << far[0] << ", " << far[1] << ", " << far[2] << ", " << far[3] << endl;
				}
				else {
					cout << "\n (No other solution)" << endl;
				}
				MoveToConfiguration(near);
				//DisplayConfiguration(near);

				
			}
			else {
				cout << "\nBoth solutions are invalid!" << endl;
			}

			cout << "Grab object? [g] or release object [r], or press any other key to do nothing" << endl;
			input = _getch();

			if (input == 'g') {
				cout << "Closing gripper " << endl;
				Grasp(true);
			}
			else if (input == 'r') {
				cout << "Opening gripper " << endl;
				Grasp(false);
			}
			else {
				//nothing
			}
			//-----
			//-----

		}

		if (input == 't') {

			ofstream p("C:/Users/jkshe/OneDrive/Documents/ENSC 488/extraProjectMatlab/pos.csv");
			ofstream v("C:/Users/jkshe/OneDrive/Documents/ENSC 488/extraProjectMatlab/vel.csv");
			ofstream accel("C:/Users/jkshe/OneDrive/Documents/ENSC 488/extraProjectMatlab/acc.csv");
			ofstream vias("C:/Users/jkshe/OneDrive/Documents/ENSC 488/extraProjectMatlab/via.csv");

			int viaPoints = 3; 
			vec via1; 
			vec via2; 
			vec via3; 
			vec goal; 
			double t; 
			double partT; 

			for (int i = 0; i < viaPoints; i++) {
				cout << "Via Point " << i + 1 << ":" << endl;
				cout << "   X(mm): ";
				if (i == 0)
					cin >> via1[0];
				else if (i == 1)
					cin >> via2[0];
				else
					cin >> via3[0];
				cout << "   Y(mm): ";
				if (i == 0)
					cin >> via1[1];
				else if (i == 1)
					cin >> via2[1];
				else
					cin >> via3[1];
				cout << "   Z(mm): ";
				if (i == 0)
					cin >> via1[2];
				else if (i == 1)
					cin >> via2[2];
				else
					cin >> via3[2];
				cout << "   Phi(deg): ";
				if (i == 0)
					cin >> via1[3];
				else if (i == 1)
					cin >> via2[3];
				else
					cin >> via3[3];
			}
			
			cout << "Goal: " << endl;
			cout << "   X(mm): ";
			cin >> goal[0];
			cout << "   Y(mm): ";
			cin >> goal[1];
			cout << "   Z(mm): ";
			cin >> goal[2];
			cout << "   Phi(deg): ";
			cin >> goal[3];

			cout << "Enter total time for trajectory: "; 
			cin >> t; 
			partT = t / (4);

			GetConfiguration(current);

			// Perform inverse kinematics
			
			/*viaInvKin(via1, via1Inv, via1S);
			for (int i = 0; i < MVSIZE; i++) {
				cout << via1Inv[i]; 
			}*/
			//cout << endl; 
			viaInvKin(via1, via1Inv, via1S);
			viaInvKin(via2, via2Inv, via2S);
			viaInvKin(via3, via3Inv, via3S);
			viaInvKin(goal, goalInv, goalS);

			if (via1S == false || via2S == false || via3S == false || goalS == false) {
				if (via1S == false) {
					cout << "\nVia 1: Has no solution.\n";
				}
				if (via2S == false) {
					cout << "Via 2: Has no solution.\n";
				}
				if (via3S == false) {
					cout << "Via 3: Has no solution.\n"; 
				}
				if (goalS == false) {
					cout << " Goal: Has no solution.\n ";
				}
			}

			// Solution found using inverse kinematics

			if (via1S != false && via2S != false && via3S != false && goalS != false) {

				CUBCOEF(current, via1Inv, v0, partT, CC1, 1); 
				//cout << "Cubic Coefficient Matrix: Start to Via1" << endl;
				//printMatrix(CC1); 
				 
				trajectoryPlanner(CC1, partT, solutions); 
				vec position1 = { solutions[0], solutions[1], solutions[2], solutions[3] };
				vec v01 = { solutions[4], solutions[5], solutions[6], solutions[7] };
				vec a1 = { solutions[8], solutions[9], solutions[10], solutions[11] };
				bool positionFlag1 = false; 
				plotter(partT, CC1, 0, positionFlag1, flags[0], p, v, accel, vias); 
				//flags[0] = solutions[12]; 


				CUBCOEF(via1Inv, via2Inv, v01, partT, CC2, 1);
				//cout << "Cubic Coefficient Matrix: Via1 to Via2" << endl;
				//printMatrix(CC2);

				trajectoryPlanner(CC2, partT, solutions);
				vec position2 = { solutions[0], solutions[1], solutions[2], solutions[3] };
				vec v12 = { solutions[4], solutions[5], solutions[6], solutions[7] };
				vec a2 = { solutions[8], solutions[9], solutions[10], solutions[11] };
				bool positionFlag2 = false;
				plotter(partT, CC2, partT, positionFlag2, flags[1], p, v, accel, vias);
				//flags[1] = solutions[12];

				CUBCOEF(via2Inv, via3Inv, v12, partT, CC3, 1);
				//cout << "Cubic Coefficient Matrix: Via2 to Via3" << endl;
				//printMatrix(CC3);
				
				trajectoryPlanner(CC3, partT, solutions);
				vec position3 = { solutions[0], solutions[1], solutions[2], solutions[3] };
				vec v23 = { solutions[4], solutions[5], solutions[6], solutions[7] };
				vec a3 = { solutions[8], solutions[9], solutions[10], solutions[11] };
				bool positionFlag3 = false;
				plotter(partT, CC3, partT*2, positionFlag3, flags[2], p, v, accel, vias);
				//flags[2] = solutions[12];

				CUBCOEF(via3Inv, goalInv, v23, partT, CC4, 1);
				//cout << "Cubic Coefficient Matrix: Via3 to Goal" << endl;
				//printMatrix(CC4);

				trajectoryPlanner(CC4, partT, solutions);
				vec position4 = { solutions[0], solutions[1], solutions[2], solutions[3] };
				vec v34 = { solutions[4], solutions[5], solutions[6], solutions[7] };
				vec a4 = { solutions[8], solutions[9], solutions[10], solutions[11] };
				bool positionFlag4 = false;
				plotter(partT, CC4, partT*3, positionFlag4, flags[3], p, v, accel, vias);
				//flags[3] = solutions[12];

				//cout << "done" << endl;
				//cout << flags[0] << flags[1] << flags[2] << flags[3] << endl;

				if (flags[0] || flags[1] || flags[2] || flags[3] || positionFlag1 || positionFlag2 || positionFlag3 || positionFlag4) {
					cout << "A violation has occured, we will not be moving the robot." << endl;
				}

				else {
					// Moving the Robot

					moveRobot(partT, CC1, 0);
					moveRobot(partT, CC2, partT);
					moveRobot(partT, CC3, partT * 2);
					moveRobot(partT, CC4, partT * 3); 

					/*MoveWithConfVelAcc(position1, v01, a1);
					sleepSeconds(partT*1000);
					MoveWithConfVelAcc(position1, v0, a1);

					MoveWithConfVelAcc(position2, v12, a2);
					sleepSeconds(partT*1000);
					MoveWithConfVelAcc(position2, v0, a2);

					MoveWithConfVelAcc(position3, v23, a3);
					sleepSeconds(partT*1000);
					MoveWithConfVelAcc(position3, v0, a3);

					MoveWithConfVelAcc(position4, v34, a4);
					sleepSeconds(partT*1000);
					MoveWithConfVelAcc(position4, v0, a4);*/

					cout << "Robot Moved, Done!" << endl;
					printL();
					
					////MoveToConfiguration(position4);

				}

				vias << current[0] << "," << via1Inv[0] << "," << via2Inv[0] << "," << via3Inv[0] << "," << goalInv[0] << endl;
				vias << current[1] << "," << via1Inv[1] << "," << via2Inv[1] << "," << via3Inv[1] << "," << goalInv[1] << endl;
				vias << current[2] << "," << via1Inv[2] << "," << via2Inv[2] << "," << via3Inv[2] << "," << goalInv[2] << endl;
				vias << current[3] << "," << via1Inv[3] << "," << via2Inv[3] << "," << via3Inv[3] << "," << goalInv[3] << endl;
				vias << partT << endl;

				p.close(); 
				v.close(); 
				accel.close(); 
				vias.close();

				cout << "Grab object? [g] or release object [r], or press any other key to do nothing" << endl;
				input = _getch();

				if (input == 'g') {
					cout << "Closing gripper " << endl;
					Grasp(true);
				}
				else if (input == 'r') {
					cout << "Opening gripper " << endl;
					Grasp(false);
				}
				else {
					//nothing
				}
			}
			


		}

		

		cout << endl;
		printf("Enter ESC to exit (enter anything else to continue)\n");
		c = _getch();

		/*for (int i = 0; i < 4; i++) {
			current[i] = q1[i];
		}*/

		q1[0] = 1000;
		q1[1] = 1000;
		q1[2] = 1000;
		q1[3] = 1000;

		input = '0';

	}

		

	return 0;
}
