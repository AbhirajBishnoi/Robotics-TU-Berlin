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

void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
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

        float PI = 3.14159265;
        float r1 = R2;
        float r2 = 0.189738;
        float r3 = R6;
        float l1 = L2;
        float l2 = L3;
        float l3 = L6;
        float m1 = M2;
        float m2 = M3+M4+M5;
        float m3 = M6;
        float c123 = cos(q1+q2+q3 - PI/2)*r3;
        float c2 = cos(q2);
        float c1 = cos(q1);
        float c12 = cos(q1 + q2 - PI/2);


        g123[0]= (r1*c1*m1+(l1*c1+r2*c12)*m2+(l1*c1+l2*c12+c123)*m3)*GRAVITY;
        g123[1]= ((c12*r2)*m2 +(l2*c12 + c123)*m3)*GRAVITY;
        g123[2]= c123 * GRAVITY;

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

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj2Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj3Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{


    //maps the torques to the right joint indices depending on the current mode:
    if (gv.dof == 3) {
        gv.tau[0] =  gv.G[0];
        gv.tau[1] =  gv.G[1];
        gv.tau[2] =  gv.G[2];
    } else if (gv.dof == 6) {
        gv.tau[1] = gv.G[1];
        gv.tau[2] = gv.G[2];
        gv.tau[4] = gv.G[4];
    }

	PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njmoveControl(GlobalVariables& gv)
{
    gv.tau[0] = gv.kp[0]*(gv.qd[0]-gv.q[0])  ;
    gv.tau[1] = gv.kp[1]*(gv.qd[1]-gv.q[1]) ;
    gv.tau[2] = gv.kp[2]*(gv.qd[2]-gv.q[2]);

    //PrintDebug(gv);

    //floatControl(gv);  // Remove this line when you implement openControl
}

void jmoveControl(GlobalVariables& gv)
{
    //controlfunction f = -kp*(x-xd) - kv 
   floatControl(gv);  // Remove this line when you implement openControl
}

void njgotoControl(GlobalVariables& gv) 
{

    gv.tau[0] = gv.kp[0]*(gv.qd[0]-gv.q[0]) + gv.G[0];//-gv.kv[0]*gv.dq[0];
    gv.tau[1] = gv.kp[1]*(gv.qd[1]-gv.q[1]) + gv.G[1];//-gv.kv[1]*gv.dq[1];
    gv.tau[2] = gv.kp[2]*(gv.qd[2]-gv.q[2]) + gv.G[2];
    //floatControl(gv);  // Remove this line when you implement openControl
}

void jgotoControl(GlobalVariables& gv) 
{
    gv.tau[0] = gv.kp[0]*(gv.qd[0]-gv.q[0])-gv.kv[0]*gv.dq[0] + gv.G[0];
    gv.tau[1] = gv.kp[1]*(gv.qd[1]-gv.q[1])-gv.kv[1]*gv.dq[1] + gv.G[1];
    gv.tau[2] = gv.kp[2]*(gv.qd[2]-gv.q[2])-gv.kv[2]*gv.dq[2] + gv.G[2];
   //floatControl(gv);  // Remove this line when you implement openControl
}

void njtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void proj1Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj1Control
}

void proj2Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
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

int XPrintf( const char* fmt, ... )
{
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
