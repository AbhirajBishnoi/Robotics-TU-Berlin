/*

 Please fill in the function stubs provided in this file

 For achieving full points on this exercise, explain the implemented equations well using comments. Especially:
   - which parameters were used for computation?
   - were equations simplified? What was the original equation?
   - when introducing additional variables, what do they contain?


 * Do not use additional includes
 * Do not include/link additional files
 * Do not change the predefined function signature

 Tip: use the main() function to test your implementation.

*/
#define _USE_MATH_DEFINES
#include "forwardkinematics.hpp" //Have a look at this header file, it declares the class ForwardKinematicsPuma2D
#include <cmath>                 //use sin, cos from cmath for your computation
#include <iostream>
using namespace std;


#ifdef UNITTEST
#define main STUDENTS_MAIN
#endif   


/*
Convenience function to print out a homogenous transform
*/
void print_HTransform(HTransform tf)
{
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout.precision(3);
	cout << "\n---------------------------\n";

	cout << tf[0][0] << "  " << tf[0][1] << "  " << tf[0][2] << "  " << tf[0][3] << endl;
	cout << tf[1][0] << "  " << tf[1][1] << "  " << tf[1][2] << "  " << tf[1][3] << endl;
	cout << tf[2][0] << "  " << tf[2][1] << "  " << tf[2][2] << "  " << tf[2][3] << endl;
	cout << tf[3][0] << "  " << tf[3][1] << "  " << tf[3][2] << "  " << tf[3][3] << endl;
	cout << "---------------------------\n";
}

/*
Convenience function to print out a 3x3 Jacobian matrix
*/
void print_Jacobian(float Jacobian[3][3])
{
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout.precision(3);
	cout << "\n+++++++++++++++++++++++++++\n";
	cout << Jacobian[0][0] << "  " << Jacobian[0][1] << "  " << Jacobian[0][2] << endl;
	cout << Jacobian[1][0] << "  " << Jacobian[1][1] << "  " << Jacobian[1][2] << endl;
	cout << Jacobian[2][0] << "  " << Jacobian[2][1] << "  " << Jacobian[2][2] << endl;
	cout << "+++++++++++++++++++++++++++\n";
}


/*
Convenience function to print out a position
*/
void print_Position(float F[3])
{
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout.precision(3);
	cout << "\nooooooooooooooooooooooooooo\n";
	cout << F[0] << "  " << F[1] << "  " << F[2] << endl;
	cout << "ooooooooooooooooooooooooooo\n";
}


/*
already implemented

Set the robot's joint values and recompute the forward kinematics.

a1, a2 and a3 are assumed to be given in radians!
*/
void ForwardKinematicsPuma2D::setJoints(float a1, float a2, float a3)
{
	//store joint angles
	angles[0] = a1;
	angles[1] = a2;
	angles[2] = a3;
	//recompute the dependent variables
	computeT0_1();
	computeT1_2();
	computeT2_3();
	computeT3_E();
	computeT0_E();
	computeF();
	computeJ();
}




/***********************************************/
/******************EDIT BELOW ******************/
/***********************************************/



/*
updates the variable T0_1

The homogenous transformation derived from translation and rotation matrix;
			[0]                           [1]                    [2]                [3]
[0]     cos(theta_i)                -sin(theta_i)                 0              alpha_i-1
[1]sin(theta_i)*cos(alpha_i-1) cos(theta_i)*cos(alpha_i-1) -sin(alpha_i-1) -sin(alpha_i-1)*d_i
[2]sin(theta_i)*sin(alpha_i-1) cos(theta_i)*sin(alpha_i-1)  cos(alpha_i-1)  cos(alpha_i-1)*d_i
[3]         0                              0                      0                  1

theta_i, alpha_i-1, d_i is used to compute homogenous transformation
*/
void ForwardKinematicsPuma2D::computeT0_1()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3

	//row vector
	T0_1[0][0] = cos(angles[0]);
	T0_1[0][1] = -sin(angles[0]);
	T0_1[0][2] = 0.0;
	T0_1[0][3] = 0.0;

	//row vector
	T0_1[1][0] = sin(angles[0]);
	T0_1[1][1] = cos(angles[0]);
	T0_1[1][2] = 0.0;
	T0_1[1][3] = 0.0;

	//row vector
	T0_1[2][0] = 0.0;
	T0_1[2][1] = 0.0;
	T0_1[2][2] = 1.0;
	T0_1[2][3] = 0.0;

	//row vector
	T0_1[3][0] = 0.0;
	T0_1[3][1] = 0.0;
	T0_1[3][2] = 0.0;
	T0_1[3][3] = 1.0;
}


/*
updates the variable T1_2

The homogenous transformation derived from translation and rotation matrix;
			[0]                           [1]                    [2]                [3]
[0]     cos(theta_i)                -sin(theta_i)                 0              alpha_i-1
[1]sin(theta_i)*cos(alpha_i-1) cos(theta_i)*cos(alpha_i-1) -sin(alpha_i-1) -sin(alpha_i-1)*d_i
[2]sin(theta_i)*sin(alpha_i-1) cos(theta_i)*sin(alpha_i-1)  cos(alpha_i-1)  cos(alpha_i-1)*d_i
[3]         0                              0                      0                  1

theta_i, alpha_i-1, d_i is used to compute homogenous transformation
*/
void ForwardKinematicsPuma2D::computeT1_2()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3

	//row vector
	T1_2[0][0] = sin(angles[1]);
	T1_2[0][1] = cos(angles[1]);
	T1_2[0][2] = 0.0;
	T1_2[0][3] = l1;

	//row vector
	T1_2[1][0] = -cos(angles[1]);
	T1_2[1][1] = sin(angles[1]);
	T1_2[1][2] = 0.0;
	T1_2[1][3] = 0.0;

	//row vector
	T1_2[2][0] = 0.0;
	T1_2[2][1] = 0.0;
	T1_2[2][2] = 1.0;
	T1_2[2][3] = 0.0;

	//row vector
	T1_2[3][0] = 0.0;
	T1_2[3][1] = 0.0;
	T1_2[3][2] = 0.0;
	T1_2[3][3] = 1.0;
}

/*
updates the variable T2_3

The homogenous transformation derived from translation and rotation matrix;
			[0]                           [1]                    [2]                [3]
[0]     cos(theta_i)                -sin(theta_i)                 0              alpha_i-1
[1]sin(theta_i)*cos(alpha_i-1) cos(theta_i)*cos(alpha_i-1) -sin(alpha_i-1) -sin(alpha_i-1)*d_i
[2]sin(theta_i)*sin(alpha_i-1) cos(theta_i)*sin(alpha_i-1)  cos(alpha_i-1)  cos(alpha_i-1)*d_i
[3]         0                              0                      0                  1

theta_i, alpha_i-1, d_i is used to compute homogenous transformation
*/
void ForwardKinematicsPuma2D::computeT2_3()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3

	//row vector
	T2_3[0][0] = cos(angles[2]);
	T2_3[0][1] = -sin(angles[2]);
	T2_3[0][2] = 0.0;
	T2_3[0][3] = l2;

	//row vector
	T2_3[1][0] = sin(angles[2]);
	T2_3[1][1] = cos(angles[2]);
	T2_3[1][2] = 0.0;
	T2_3[1][3] = 0.0;

	//row vector
	T2_3[2][0] = 0.0;
	T2_3[2][1] = 0.0;
	T2_3[2][2] = 1.0;
	T2_3[2][3] = 0.0;

	//row vector
	T2_3[3][0] = 0.0;
	T2_3[3][1] = 0.0;
	T2_3[3][2] = 0.0;
	T2_3[3][3] = 1.0;
}


/*
updates the variable T3_E

The homogenous transformation derived from translation and rotation matrix;
			[0]                           [1]                    [2]                [3]
[0]     cos(theta_i)                -sin(theta_i)                 0              alpha_i-1
[1]sin(theta_i)*cos(alpha_i-1) cos(theta_i)*cos(alpha_i-1) -sin(alpha_i-1) -sin(alpha_i-1)*d_i
[2]sin(theta_i)*sin(alpha_i-1) cos(theta_i)*sin(alpha_i-1)  cos(alpha_i-1)  cos(alpha_i-1)*d_i
[3]         0                              0                      0                  1

theta_i, alpha_i-1, d_i is used to compute homogenous transformation
*/
void ForwardKinematicsPuma2D::computeT3_E()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3

	//row vector
	T3_E[0][0] = 1.0;
	T3_E[0][1] = 0.0;
	T3_E[0][2] = 0.0;
	T3_E[0][3] = l3;

	//row vector
	T3_E[1][0] = 0.0;
	T3_E[1][1] = 1.0;
	T3_E[1][2] = 0.0;
	T3_E[1][3] = 0.0;

	//row vector
	T3_E[2][0] = 0.0;
	T3_E[2][1] = 0.0;
	T3_E[2][2] = 1.0;
	T3_E[2][3] = 0.0;

	//row vector
	T3_E[3][0] = 0.0;
	T3_E[3][1] = 0.0;
	T3_E[3][2] = 0.0;
	T3_E[3][3] = 1.0;
}


/*

This function updates the variable T0_E

The homogenous transformation derived from translation and rotation matrix;
			[0]                           [1]                    [2]                [3]
[0]     cos(theta_i)                -sin(theta_i)                 0              alpha_i-1
[1]sin(theta_i)*cos(alpha_i-1) cos(theta_i)*cos(alpha_i-1) -sin(alpha_i-1) -sin(alpha_i-1)*d_i
[2]sin(theta_i)*sin(alpha_i-1) cos(theta_i)*sin(alpha_i-1)  cos(alpha_i-1)  cos(alpha_i-1)*d_i
[3]         0                              0                      0                  1


After calculate homogenous transformation matrix for each link.
The transformation matrix from 0 to End effector is a multiplication of each matrix

theta_i, alpha_i-1, d_i is used to compute homogenous transformation

*/
void ForwardKinematicsPuma2D::computeT0_E()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3


	//row vector
	T0_E[0][0] = sin(angles[0] + angles[1] + angles[2]);
	T0_E[0][1] = cos(angles[0] + angles[1] + angles[2]);
	T0_E[0][2] = 0.0;
	T0_E[0][3] = sin(angles[0] + angles[1] + angles[2])*l3 + sin(angles[0] + angles[1])*l2 + cos(angles[0])*l1;

	//row vector
	T0_E[1][0] = -cos(angles[0] + angles[1] + angles[2]);
	T0_E[1][1] = sin(angles[0] + angles[1] + angles[2]);
	T0_E[1][2] = 0.0;
	T0_E[1][3] = -cos(angles[0] + angles[1] + angles[2])*l3 - cos(angles[0] + angles[1])*l2 + sin(angles[0])*l1;

	//row vector
	T0_E[2][0] = 0.0;
	T0_E[2][1] = 0.0;
	T0_E[2][2] = 1.0;
	T0_E[2][3] = 0.0;

	//row vector
	T0_E[3][0] = 0.0;
	T0_E[3][1] = 0.0;
	T0_E[3][2] = 0.0;
	T0_E[3][3] = 1.0;
}



/*
This function updates the variables ee_x, ee_y, ee_alpha

F is an unit vector contains the position and orientation of end-effector.
ee_x is a translation in X axis
ee_y is a translation in Y axis
ee_alpha is an angle in the rotational matrix

*/
void ForwardKinematicsPuma2D::computeF()
{

	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3

	F[0] = T0_E[0][3]; //x
	F[1] = T0_E[1][3]; //y
	F[2] = angles[0] + angles[1] - M_PI_2 + angles[2]; //alpha
}


/*
This function updates the variable J

Jacobian matrix is derived from the derivative as follows:
dF_x(angle_i)/dangle_i
dF_y(angle_i)/dangle_i
dF_alpha(angle_i)/dangle_i

where i is a joint
*/
void ForwardKinematicsPuma2D::computeJ()
{
	// compute from 
	// angles[0], angles[1], angles[2], l1, l2, and l3
	J[0][0] = (-l1 * sin(angles[0])) + (l2*cos(angles[0] + angles[1])) + (l3*cos(angles[0] + angles[1] + angles[2]));
	J[0][1] = (l2*cos(angles[0] + angles[1])) + (l3*cos(angles[0] + angles[1] + angles[2]));
	J[0][2] = (l3*cos(angles[0] + angles[1] + angles[2]));

	//row vector
	J[1][0] = (l1*cos(angles[0])) + (l2*sin(angles[0] + angles[1])) + (l3*sin(angles[0] + angles[1] + angles[2]));
	J[1][1] = (l2*sin(angles[0] + angles[1])) + (l3*sin(angles[0] + angles[1] + angles[2]));
	J[1][2] = (l3*sin(angles[0] + angles[1] + angles[2]));

	//row vector
	J[2][0] = 1.0;
	J[2][1] = 1.0;
	J[2][2] = 1.0;
}

   

/*
Example code to test your functions:

You are free to change main() as you like
*/
int main()
{
	ForwardKinematicsPuma2D* fk = new ForwardKinematicsPuma2D();
	fk->setJoints(0.0, 0.0, 0.0);
	print_HTransform(fk->T0_E); //example
	print_Position(fk->F); //example
	print_Jacobian(fk->J); //example

	ForwardKinematicsPuma2D* fk2 = new ForwardKinematicsPuma2D();
	fk2->setJoints(M_PI_2, -M_PI_2, 0.0);
	print_HTransform(fk2->T0_E); //example
	print_Position(fk2->F); //example
	print_Jacobian(fk2->J); //example

	ForwardKinematicsPuma2D* fk3 = new ForwardKinematicsPuma2D();
	fk3->setJoints(0, M_PI_2, 0.01);
	print_HTransform(fk3->T0_E); //example
	print_Position(fk3->F); //example
	print_Jacobian(fk3->J); //example
	system("pause");
	return 0;
}

