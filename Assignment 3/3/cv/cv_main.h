#ifndef CV_MAIN_H
#define CV_MAIN_H
#include "GlobalVariables.h"
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//use this to declare publicly available functions conforming to a C ABI:
#define PUBLIC extern "C" __attribute__ ((visibility ("default"))) 

// Minimum elapsed time between commands sent by the visual servoing loop [seconds]
// (provides additional safety)
#define COMMAND_SEND_MIN_TIME 0.08

//Target period for the Vision loop:
#define VISIONLOOP_PERIOD 0.1

typedef void (*cmdfunc)(const char*);
typedef cv::Mat* (*imgFactory)(void);



/* 
* cvLoop
* Computer Vision Loop: the images are grabbed and analyzed 
* The loop will end if the main window is closed or cvStop is called
* The main program creates a thread (the computer vision thread) that executes this function until it ends
* @param robotCmd - Robot command to be called. A robot command is the name of a control function and a list of arguments for this function, all in a array of chars. 
* E.g.: "jgoto 0.0 0.0 0.0 0.0 0.0 0.0"
* @param getSimImg - Function to be called to obtain images in simulation
*
* extern -> It can be called as a function of the library
*/
PUBLIC void cvLoop(cmdfunc robotCmd, imgFactory getSimImg);

/* 
* cvStop
* Callback when the user presses the button Stop CV on the GUI
* Set the running flag to false to stop the cvLoop execution
* extern -> It can be called as a function of the library
*/
PUBLIC void cvStop(void);

/* 
* cvSetRobotState
* Callback function to inform about the current robot state, so that this state can be used in the Computer Vision Loop
* @param gv - Current state of the robot
* extern -> It can be called as a function of the library
*/
PUBLIC void cvSetRobotState(GlobalVariables &gv);

/*
* on_mouse
* Callback event called when we click with the mouse on the image
* We use it to click with the left button on the image and select a rectangle that contains the color we want to track
*/
void onMouse( int event, int x, int y, int flags, void* param );


#endif
