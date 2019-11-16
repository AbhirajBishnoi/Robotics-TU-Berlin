#include "cv_control.h"



using namespace cv;

using namespace std;



// Parameters for Visual Servoing

float f;					// Focal length in pixel

float img_width;			// width of image sensor in pixel

float img_height;			// height of image sensor in pixel

float diameter_real;		// real diameter of the circle

float diameter_desired_px;	// desired diameter of the circle in pixels

float dt;					// Time step

float t0;                  // how fast the velocity controller should converge

int   hcd_min_distance;

PrVector3 desired_s_opencvf;



void initVisualServoing(float _f, float _img_width, float _img_height, float _diameter_real, float _diameter_desired_px, float _dt, PrVector3 _desired_s_opencvf)

{

	f = _f;

	img_width = _img_width;

	img_height = _img_height;

	diameter_real = _diameter_real;

	diameter_desired_px = _diameter_desired_px;

	dt = _dt;

	desired_s_opencvf = _desired_s_opencvf;

	t0 = 0.3;   //time constant for controller convergence. Smaller values result in higher velocities. Transforms an error into a velocity

	hcd_min_distance = 2000;

}



/*****************************************************************************************************************/

/* YOUR WORK STARTS HERE!!! */



/**

* findCircleFeature

* Find circles in the image using the OpenCV Hough Circle detector

*

* Input parameters:

*  img: the camera image, you can print text in it with

* 	    putText(img,"Hello World",cvPoint(0,12),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,0,255))

*	    see http://opencv.willowgarage.com/documentation/cpp/drawing_functions.html#cv-puttext

*

*  backproject: grayscale image with high values where the color of the image is like the selected color.

*

* Output:

*  crcl: as a result of this function you should write the center and radius of the detected circle into crcl

*/

bool findCircleFeature(Mat& img, Mat &backproject, Circle& crcl)

{
	// create vector of vectors containing 3 floats to store all circles
	vector<Vec3f> circles;

	// smooth the edges in the image with gaussian blur for pixeled pictures
	GaussianBlur(backproject, backproject, Size(9, 9), 2, 2);

	// function to detect the circles
	HoughCircles(backproject, circles, CV_HOUGH_GRADIENT, 1, backproject.rows / 8, 150, 45, 0, 0);

	if (circles.size() == 0) {
		// return false when no circles are detected
		return false;

	}
	else {
		// variables to find the biggest circle
		int index = 0;
		int max_radius = 0;

		for (size_t i = 0; i < circles.size(); i++)
		{
			// create a point just for the center and store the radius
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// draw circle outline in original image
			circle(img, center, radius, Scalar(0, 0, 255), 3, 8, 0);
			// store the biggest circles index and radius
			if (max_radius < radius) {
				index = i;
				max_radius = radius;
			}
		}

		// fill the circle gv with the values from the biggest circle
		crcl.center.x = circles[index][0];
		crcl.center.y = circles[index][1];
		crcl.radius = circles[index][2];

		return true;

	}

}



/**

* getImageJacobianCFToFF

* Compute the Image Jacobian to map from

* camera velocity in Camera Frame to feature velocities in Feature Frame

*

* You should use getImageJacobianFFToCF in controlRobot

*

* Input parameters:

*  u and v: the current center of the circle in feature frame [pixels]

*  z: the estimated depth of the circle [meters]

*  f: the focal length [pixels]

*  diameter: the real diameter of the circle [meters]

*

* Output:

*  Jv: assign your image 3x3 Jacobian.

*/

void getImageJacobianCFToFF(PrMatrix3 &Jv, float u, float v, float z, float f, float diameter)

{

	// du/dt = f/z*dx/dt - f*x/z^2*dz/dt;
	// dv/dt = f/z*dy/dt - f*y/z^2*dz/dt;
	// dd/dt = - f*D/z^2*dz/dt;

    // Because what we move is camera not image
    // the velocities should be negative
    // so the sign should be opposite


	Jv[0][0] = -f / z;
	Jv[0][1] = 0;
	Jv[0][2] = u / z;

	Jv[1][0] = 0;
	Jv[1][1] = -f / z;
	Jv[1][2] = v / z;

	Jv[2][0] = 0;
	Jv[2][1] = 0;
	Jv[2][2] = f * diameter / pow(z, 2);

}



/**

* estimateCircleDepth

* Estimates and returns the depth of the circle

*

* Input parameters:

*  f: the focal length [pixels]

*  diameter: the real diameter of the circle [meters]

*  crcl: the parameters of the detected circle in the image

*

* Output return:

*  depth of the circle wrt the camera [meters]

*/

float estimateCircleDepth(float f, float diameter, Circle &crcl)

{

	return f * diameter / (2 * crcl.radius);  //diameter/depth = 2*crcl.radius/f

}



/**

* transformFromOpenCVFToFF

* Transform a feature vector from openCV frame (origin in upper left corner of the image) to feature frame (origin at the center of the image)

*

* Input parameter:

*  vector_opencvf: feature vector defined in opencv frame

*

* Output:

*  vector_ff: feature vector defined in feature frame

*/

void transformFromOpenCVFToFF(PrVector3 vector_opencvf, PrVector3& vector_ff)

{

	// x in ff = x in opencvf - half the width of the ff

	// y in ff = y in opencvf - half the height of the ff

	// z is the same in both frames

	vector_ff[0] = vector_opencvf[0] - img_width / 2;

	vector_ff[1] = vector_opencvf[1] - img_height / 2;

	vector_ff[2] = vector_opencvf[2];

}



/**

* transformVelocityFromCFToEEF

* Transform the desired velocity vector from camera frame to end-effector frame

* You can hard code this transformation according to the fixed transformation between the camera and the end effector

* (see the sketch in your assignment)

*

* Input parameter:

*  vector_cf: velocity vector defined in camera frame

*

* Output:

*  vector_eef: velocity vector defined in end-effector frame

*/

void transformVelocityFromCFToEEF(PrVector3 vector_cf, PrVector3& vector_eef)

{

	// | vector_eef[0] |      |0 -1 0|   |vector_cf[0]|

	// | vector_eef[1] |  =   |1  0 0| * |vector_cf[1]|

	// | vector_eef[2] |      |0  0 1|   |vector_cf[2]|



	vector_eef[0] = -vector_cf[1];

	vector_eef[1] = vector_cf[0];

	vector_eef[2] = vector_cf[2];

}



/**

* transformVelocityFromEEFToBF

* Transform the desired velocity vector from end-effector frame to base frame

* You cannot hard code this transformation because it depends of the current orientation of the end-effector wrt the base

* Make use of the current state of the robot x (the pose of the end-effector in base frame coordinates)

*

* Input parameters:

*  x_current_bf: current state of the robot - pose of the end-effector in base frame coordinates

*  vector_eef: velocity vector defined in end-effector frame

*

* Output:

*  vector_bf: velocity vector defined in base frame

*/

void transformVelocityFromEEFToBF(PrVector x_current_bf, PrVector3 vector_eef, PrVector3& vector_bf)

{
    // Velocities have no position, but rotation does
    // so what we need to do is rotate the frame
    // we have already the quaternion of EEFT in BF
    // we can  convert it to rotationmatrix
    // then use the transformmatrix to rotate the frame
	
	// convert quaternion to rotationmatrix

	PrMatrix3 Tr;


	Tr[0][0] = 1 - 2 * pow(x_current_bf[5], 2) - 2 * pow(x_current_bf[6], 2);

	Tr[0][1] = 2 * x_current_bf[4] * x_current_bf[5] - 2 * x_current_bf[3] * x_current_bf[6];

	Tr[0][2] = 2 * x_current_bf[4] * x_current_bf[6] + 2 * x_current_bf[3] * x_current_bf[5];



	Tr[1][0] = 2 * x_current_bf[4] * x_current_bf[5] + 2 * x_current_bf[3] * x_current_bf[6];

	Tr[1][1] = 1 - 2 * pow(x_current_bf[4], 2) - 2 * pow(x_current_bf[6], 2);

	Tr[1][2] = 2 * x_current_bf[5] * x_current_bf[6] - 2 * x_current_bf[3] * x_current_bf[4];



	Tr[2][0] = 2 * x_current_bf[4] * x_current_bf[6] - 2 * x_current_bf[3] * x_current_bf[5];

	Tr[2][1] = 2 * x_current_bf[5] * x_current_bf[6] + 2 * x_current_bf[3] * x_current_bf[4];

	Tr[2][2] = 1 - 2 * pow(x_current_bf[4], 2) - 2 * pow(x_current_bf[5], 2);



	vector_bf = Tr * vector_eef;

}



/*

* controlRobot

* This function computes the command to be send to the robot using Visual Servoing so that the robot tracks the circle

*

* Here you should:

* - compute the error in feature frame

* - compute the circle depth

* - compute the image jacobian from feature frame in camera frame

* - compute the desired ee velocity in feature frame

* - compute the desired ee velocity in camera frame

* - compute the desired ee velocity in ee frame

* - compute the desired ee velocity in base frame

* - compute the step in the direction of the desired ee velocity in base frame

* - form the comand to be sent to the robot (previous pose + computed step)

*

* The function will only be called if findCircleFeature returns true (if a circle is detected in the image)

*

* Input parameters:

*  crcl: the parameters of the detected circle in the image

*  x:	current robot configuration in operational space (7 dof: 3 first values are position, 4 last values is orientation quaternion)

*  img: the camera image for drawing debug text

*

* Output:

*  cmdbuf: should contain the command for the robot controler, for example:

*			"goto 0.0 0.0 90.0 0.0 0.0 0.0"

*/



void controlRobot(Circle& crcl, PrVector &x, Mat& img, char *cmdbuf)

{

	if (crcl.radius == 0) {

		sprintf(cmdbuf, "float");

		return;

	}



	PrVector3 current_s_opencvf;

	current_s_opencvf[0] = crcl.center.x;

	current_s_opencvf[1] = crcl.center.y;

	current_s_opencvf[2] = 2 * crcl.radius;



	PrVector3 desired_s_ff;

	transformFromOpenCVFToFF(desired_s_opencvf, desired_s_ff);

	PrVector3 current_s_ff;

	transformFromOpenCVFToFF(current_s_opencvf, current_s_ff);



	PrVector3 error_s_ff = desired_s_ff - current_s_ff;



	float z = estimateCircleDepth(f, diameter_real, crcl);



	PrMatrix3 Jv;

	getImageJacobianCFToFF(Jv, current_s_ff[0], current_s_ff[1], z, f, diameter_real);



	PrMatrix3 Jv_inv;

	// calculate the rank of the Jacobian 
	int rank;
	rank = Jv.rank();

	// if the rank is not full (here n = 3) use pseudoInverse to avoid singularities if it is use inverse
	if (rank == 3) {
		Jv.inverse(Jv_inv);
	}
	else {
		Jv.pseudoInverse(Jv_inv);
	}


	//Compute the desired velocity of the feature in feature frame

	PrVector3 vel_f_ff = error_s_ff / t0;



	//Compute the desired velocity of the end effector in camera frame

	PrVector3 vel_ee_cf = Jv_inv * vel_f_ff;



	PrVector3 vel_ee_eef;

	transformVelocityFromCFToEEF(vel_ee_cf, vel_ee_eef);



	PrVector3 vel_ee_bf;

	transformVelocityFromEEFToBF(x, vel_ee_eef, vel_ee_bf);



	// compute the next EE position for the next timestep given the desired EE velocity:

	PrVector3 step_ee_bf = vel_ee_bf * dt;



	PrVector desired_ee_pose_bf = x;

	// check that the robot is still inside the sphere with a radius of 85 cm around the base frame
	// calcualte length of the end effector vector without the root and compare to the squared radius

	// used the unrooted equation for length to save time while running
	float unrooted_length = pow(desired_ee_pose_bf[0] += step_ee_bf[0], 2) + pow(desired_ee_pose_bf[1] += step_ee_bf[1], 2) + pow(desired_ee_pose_bf[2] += step_ee_bf[2], 2);

	// check if unrooted_length is lower than the squared sphere radius
	if (unrooted_length < pow(0.85, 2)) {

		desired_ee_pose_bf[0] += step_ee_bf[0];

		desired_ee_pose_bf[1] += step_ee_bf[1];

		desired_ee_pose_bf[2] += step_ee_bf[2];

		// the robot only gets the command to go to a new position when it stays inside the sphere
		sprintf(cmdbuf, "goto %.4f %.4f %.4f %.4f %.4f %.4f %.4f", desired_ee_pose_bf[0], desired_ee_pose_bf[1], desired_ee_pose_bf[2], 0.50, 0.50, -0.50, 0.50);

	}



	putText(img, cmdbuf, cv::Point(5, 50), FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0, 255, 0), 1.2);

}
