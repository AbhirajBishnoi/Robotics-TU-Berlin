// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>

using std::min;
using std::max;

double timeInitial;
double currentAngularPosition;
double currentAngularVelocity;

struct CubicSpline {
double t0 , tf;
PrVector a0 , a1 , a2 , a3;
};
CubicSpline spline;


void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) {
    // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv) {
    // This code runs on every servo loop, just before the control law


    //
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        }

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here!
        float vr1 = R2;
        float vr2 = 0.189738;
        float vr3 = R6;
        float vl1 = L2;
        float vl2 = L3;
        float vl3 = L6;
        float vm1 = (M2);
        float vm2 = (M3 + M4 + M5);
        float vm3 = (M6);
        float vg = -9.81;

        float c1 = cos(q1);
        float c12 = cos(q1 + (q2 - M_PI * 0.5));
        float c23 = cos((q2 - M_PI * 0.5) + q3);

        g123[0] = vg * (vr1 * c1 * vm1 + (vl1 * c1 + vr2 * c12) * vm2 + (vl1 * c1 + vl2 * c12 + vr3 * c23) * vm3);
        g123[1] = vg * (vr2 * c12 * vm2 + (vl2 * c12 + vr3 * c23) * vm3);
        g123[2] = vg * (vr3 * c23 * vm3);

        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }

        //        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
    }
}

void PostprocessControl(GlobalVariables& gv) {
    // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initJgotoControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

double computeTf ( GlobalVariables& gv)
{
   double time[4];
   for (int i=0;i<3;i++) {
        time[i]= abs(gv.qd[i]-gv.q[i]/(gv.dqmax[i]/2)); // Time = Distance travelled / average velocity, try to use displacement to approximate distance
   }

   double *max = max_element(time,time+3);
   return *max;
}

void initNjtrackControl(GlobalVariables& gv) {
   PrVector q0 = gv.q;
   PrVector dq0 = gv.dq;
   PrVector qf = gv.qd;
   PrVector dqf = gv.dqd;

   spline.t0 = gv.curTime;
   spline.tf = computeTf(gv);
   spline.a0 = q0;
   spline.a1 = dq0;
   spline.a2 = 3*(qf-q0)/pow(spline.tf-spline.t0,2) - dqf/(spline.tf-spline.t0)- 2*dq0/(spline.tf-spline.t0);
   spline.a3 = -2*(qf-q0)/pow(spline.tf-spline.t0,3) + (dqf + dq0)/pow(spline.tf-spline.t0,2);
}

void initJtrackControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNholdControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initGotoControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initNtrackControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initPfmoveControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initLineControl(GlobalVariables& gv) {
    // Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) {
}

void initProj2Control(GlobalVariables& gv) {
    	timeInitial = gv.curTime;
}

void initProj3Control(GlobalVariables& gv) {
    	timeInitial = gv.curTime;
	currentAngularPosition = M_PI / 2;
	currentAngularVelocity = 0.0;
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv) {
}

void floatControl(GlobalVariables& gv) {
    gv.tau = gv.G;
    PrintDebug(gv);
}

void openControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void jholdControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void njmoveControl(GlobalVariables& gv) {
    gv.tau = gv.kp * (gv.qd - gv.q);
}

void jmoveControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void njgotoControl(GlobalVariables& gv) {
    gv.tau = gv.kp * (gv.qd - gv.q) + gv.G;
}

void jgotoControl(GlobalVariables& gv) {
    gv.tau = gv.kp * (gv.qd - gv.q) - gv.kv * gv.dx + gv.G;
}

void njtrackControl(GlobalVariables& gv) {
    gv.tau = -gv.kp * (gv.q - gv.qd)- gv.kv * (gv.dq - gv.dqd) + gv.G; 
}

void jtrackControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void nxtrackControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void xtrackControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void nholdControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void holdControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void ngotoControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void gotoControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void ntrackControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void pfmoveControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void lineControl(GlobalVariables& gv) {
    floatControl(gv);  // Remove this line when you implement openControl
}

void proj1Control(GlobalVariables& gv) {
	gv.qd[0] = -0.45;
	gv.qd[1] = 0.65;
	gv.qd[2] = -0.17;
	njtrackControl(gv);
}

void proj2Control(GlobalVariables& gv) {
 
	double timeCurrent = gv.curTime - timeInitial;
	double angleCurrent = (-2 * M_PI / 5 * timeCurrent) + (M_PI / 2);
	double positionX = 0.2 * cos(angleCurrent) + 0.45;
	double positionY = 0.2 * sin(angleCurrent) + 0.6;

	gv.xd = PrVector3(positionX, positionY, 0.0);

	PrVector forceCalculated = gv.kp * (gv.xd - gv.x) - gv.kv * (gv.dx - gv.dxd);

	gv.tau = gv.Jtranspose * forceCalculated + gv.G;
   
}

void proj3Control(GlobalVariables& gv) {

	double startOffset = M_PI / 2;
	double angleTotal = -6 * M_PI + startOffset;
	double timeTotal = 20;
	double timeCurrent = gv.curTime - timeInitial;
	double angleAccelerateEnd = -M_PI + startOffset;
	double angleDecelerateBegin = -5 * M_PI + startOffset;
	double maxAngularVelocity = -2 * M_PI / 5;
	double maxAngularAcceleration = -2 * M_PI / 25;


	if (timeCurrent <= timeTotal) {
		if(currentAngularPosition > angleAccelerateEnd) {
			currentAngularPosition = 0.5 * maxAngularAcceleration * pow(timeCurrent, 2) + startOffset;
			currentAngularVelocity = maxAngularAcceleration * timeCurrent;
//std::cout <<"A: currentAngularPosition=" << currentAngularPosition / M_PI << "; currentAngularVelocity=" << currentAngularVelocity/M_PI << std::endl;
		} else if (currentAngularPosition > angleDecelerateBegin) {
			currentAngularPosition = maxAngularVelocity * (timeCurrent - 5) + startOffset - M_PI;
			currentAngularVelocity = maxAngularVelocity;
//std::cout <<"B: currentAngularPosition=" << currentAngularPosition / M_PI << "; currentAngularVelocity=" << currentAngularVelocity/M_PI << "; currentTime=" << timeCurrent << std::endl;
		} else {
			currentAngularPosition = angleTotal - 0.5 * maxAngularAcceleration * pow((timeCurrent - timeTotal), 2);
			currentAngularVelocity = -maxAngularAcceleration * (timeCurrent - timeTotal);
//std::cout <<"C: currentAngularPosition=" << currentAngularPosition / M_PI << "; currentAngularVelocity=" << currentAngularVelocity/M_PI << "; currentTime=" << timeCurrent  << std::endl;
		}

		double positionX = 0.2 * cos(currentAngularPosition) + 0.45;
		double positionY = 0.2 * sin(currentAngularPosition) + 0.6;

		gv.xd = PrVector3(positionX, positionY, 0.0);

		double velocityX = -currentAngularVelocity * 0.2 * sin(currentAngularPosition);
		double velocityY = currentAngularVelocity * 0.2 * cos(currentAngularPosition);

		gv.dxd = PrVector3(velocityX, velocityY, 0.0);

		PrVector forceCalculated = gv.kp * (gv.xd - gv.x) - gv.kv * (gv.dx - gv.dxd);

		gv.tau = gv.Jtranspose * forceCalculated + gv.G;
	} else {
    		floatControl(gv);
	}
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv) {
    // Replace this code with any debug information you'd like to get
    // when you type "pdebug" at the prompt.
    printf( "This sample code prints the torque and mass\n" );
    gv.tau.display( "tau" );
    gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... ) {
    int returnValue;
    va_list argptr;
    va_start( argptr, fmt );

    returnValue = vprintf( fmt, argptr );

    va_end( argptr );
    return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

  END OF DEFAULT STUDENT FILE

  ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS

 *******************************************************/
