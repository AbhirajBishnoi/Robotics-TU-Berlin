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

/********************************************************
VISUAL SERVOING Add-on
*******************************************************/

static PrVector splineParam0;  // params used for track splines
static PrVector splineParam2;
static PrVector splineParam3;
static Float splineStartTime, splineDuration;

void FindOperationalCubicSpline(GlobalVariables& gv)
{
   splineStartTime = gv.curTime;
   splineDuration  = 0.0;

   // Find the finish time

   static const Float MIN_VMAX = 1e-4f;

   if( gv.dxmax < MIN_VMAX || gv.ddxmax < MIN_VMAX || gv.wmax < MIN_VMAX )
   {
      splineDuration = 1 / MIN_VMAX;    // Avoid divide-by-zero errors
   }
   else
   {
      PrVector deltaX = gv.Einverse * ( gv.x - gv.xd );
      Float vDuration = (gv.selectLinear * deltaX).magnitude() * 1.5 / gv.dxmax;
      Float wDuration = (gv.selectAngular * deltaX).magnitude() * 1.5 / gv.wmax;
      Float aDuration =
         sqrt( (gv.selectLinear * deltaX).magnitude() * 6.0 / gv.ddxmax );
      splineDuration = max( splineDuration, vDuration );
      splineDuration = max( splineDuration, wDuration );
      splineDuration = max( splineDuration, aDuration );
   }

   if( splineDuration < 1e-3 )
   {
      // Avoid divide-by-zero errors
      splineParam0 = gv.xd;
      splineParam2.setSize( gv.xd.size(), true );
      splineParam3.setSize( gv.xd.size(), true );
   }
   else
   {
      Float splineDuration2 = splineDuration * splineDuration;
      Float splineDuration3 = splineDuration * splineDuration2;
      splineParam0 = gv.x;
      splineParam2 = (gv.xd - gv.x) * (3.0 / splineDuration2);
      splineParam3 = (gv.xd - gv.x) * (-2.0 / splineDuration3);
   }
}

void EvaluateCubicSpline( PrVector& pos, PrVector& vel, PrVector& acc, GlobalVariables& gv )
{
   float t = gv.curTime - splineStartTime;

   if( t > splineDuration )
   {
      t = splineDuration;
      pos = splineParam0 + splineParam2 * t*t + splineParam3 * t*t*t;
      vel.setSize( splineParam0.size(), true );
      acc.setSize( splineParam0.size(), true );
   }
   else
   {
      Float t2 = t * t;
      Float t3 = t * t2;
      pos = splineParam0 + splineParam2 * t2 + splineParam3 * t3;
      vel = splineParam2 * 2.0 * t + splineParam3 * 3.0 * t2;
      acc = splineParam2 * 2.0 + splineParam3 * 6.0;
   }
}

void getSaturatedFprime( PrVector& fPrime, GlobalVariables& gv )
{
   static const Float MIN_KV = 1e-4;

   // Find the desired velocity (linear & angular)

   PrVector deltaX = gv.Einverse * ( gv.x - gv.xd );
   PrVector local_dxd = gv.dxd;
   for( int ii = 0; ii < gv.dxd.size(); ii++ )
   {
      local_dxd[ii] -= deltaX[ii] * gv.kp[ii] / max(gv.kv[ii], MIN_KV);
   }

   // Perform velocity saturation

   Float linearVelocity = (gv.selectLinear * local_dxd).magnitude();
   if( linearVelocity > gv.dxmax )
   {
      local_dxd *= gv.dxmax / linearVelocity;
   }

   Float angularVelocity = (gv.selectAngular * local_dxd).magnitude();
   if( angularVelocity > gv.wmax )
   {
      local_dxd *= gv.wmax / angularVelocity;
   }

   // Calculate f-prime

   fPrime = -gv.kv * (gv.dx - local_dxd);
}
void GetNonsingularSelection( PrMatrix& dest, GlobalVariables& gv )
{
   // Find rotations of frames 3 & 5
   //
   Float q1 = gv.q[0];
   Float q2 = gv.q[1];
   Float q3 = gv.q[2];
   Float q4 = gv.q[3];
   Float q5 = gv.q[4];
   float c1  = cos( q1 );
   float s1  = sin( q1 );
   float c23 = cos( q2 + q3 );
   float s23 = sin( q2 + q3 );
   float c4  = cos( q4 );
   float s4  = sin( q4 );
   float c5  = cos( q5 );
   float s5  = sin( q5 );

   PrMatrix3 R3( c1 * c23,  -c1 * s23,  -s1,
                 s1 * c23,  -s1 * s23,   c1,
                     -s23,       -c23,    0 );
   PrMatrix3 R35( c4 * c5,  -c4 * s5,  -s4,
                       s5,        c5,    0,
                  s4 * c5,  -s4 * s5,   c4 );
   PrMatrix3 R5 = R3 * R35;

   // Resize the selection matrix
   //
   int reducedDof = 6;
   if( gv.singularities & ELBOW_LOCK )
   {
      --reducedDof;
   }
   if( gv.singularities & HEAD_LOCK )
   {
      --reducedDof;
   }
   if( gv.singularities & WRIST_LOCK )
   {
      --reducedDof;
   }
   dest.setSize( reducedDof, 6, true );

   // Initialize selection matrix for ELBOW_LOCK and HEAD_LOCK
   //
   int row = 0;
   if ( ( gv.singularities & (ELBOW_LOCK | HEAD_LOCK) ) == 0 )
   {
      dest[row++][0] = 1;
      dest[row++][1] = 1;
      dest[row++][2] = 1;
   }
   else
   {
      PrVector3 x3( R3[0][0], R3[1][0], R3[2][0] );
      PrVector3 y3( R3[0][1], R3[1][1], R3[2][1] );
      PrVector3 z3( R3[0][2], R3[1][2], R3[2][2] );
      dest[row][0] = x3[0];
      dest[row][1] = x3[1];
      dest[row][2] = x3[2];
      row++;
      if ( ( gv.singularities & ELBOW_LOCK ) == 0 )
      {
         dest[row][0] = y3[0];
         dest[row][1] = y3[1];
         dest[row][2] = y3[2];
         row++;
      }
      if ( ( gv.singularities & HEAD_LOCK ) == 0 )
      {
         dest[row][0] = z3[0];
         dest[row][1] = z3[1];
         dest[row][2] = z3[2];
         row++;
      }
   }

   // Initialize selection matrix for WRIST_LOCK
   //
   if ( ( gv.singularities & WRIST_LOCK ) == 0 )
   {
      dest[row++][3] = 1;
      dest[row++][4] = 1;
      dest[row++][5] = 1;
   }
   else
   {
      PrVector3 y5( R5[0][1], R5[1][1], R5[2][1] );
      PrVector3 z5( R5[0][2], R5[1][2], R5[2][2] );
      dest[row][3] = y5[0];
      dest[row][4] = y5[1];
      dest[row][5] = y5[2];
      row++;
      dest[row][3] = z5[0];
      dest[row][4] = z5[1];
      dest[row][5] = z5[2];
      row++;
   }
}

void GetEscapeTorque( const PrVector& fPrime, PrVector& escapeTorque, GlobalVariables& gv )
{
   PrVector6 nullSpaceMotion;
   PrVector6 force;
   for( int ii = 0; ii < 6; ii++ )
   {
      static const Float MIN_KP = 0.1;
      force = fPrime / max( gv.kp[ii], MIN_KP );
   }

   // Find rotation of frame 4

   Float q1 = gv.q[0];
   Float q2 = gv.q[1];
   Float q3 = gv.q[2];
   Float q4 = gv.q[3];
   float c1  = cos( q1 );
   float s1  = sin( q1 );
   float c2  = cos( q2 );
   float c23 = cos( q2 + q3 );
   float s23 = sin( q2 + q3 );
   float c4  = cos( q4 );
   float s4  = sin( q4 );

   PrMatrix3 R4( c1*c23*c4 - s1*s4,  -c1*c23*s4 - s1*c4,  c1*s23,
                 s1*c23*c4 + c1*s4,  -s1*c23*s4 + c1*c4,  s1*s23,
                   -s23*c4,              s23*s4,             c23 );

   // Elbow lock

   if ( gv.singularities & ELBOW_LOCK )
   {
      // Assume that the radial force is proportional to the linear
      // force dot position
      Float radialForce =
         force[0] * gv.x[0] + force[1] * gv.x[1] + force[2] * gv.x[2];

      if ( radialForce > 0 )
      {
         // We want to move outward, so move q3 toward pi/2
         nullSpaceMotion[2] += radialForce * ( M_PI/2 - q3 );
      }
      else if ( L2 * c2 + L3 * s23 >= 0 )
      {
         // We want to move inward, and we're less likely to hit a
         // joint limit if we move the elbow up
         nullSpaceMotion[2] -= radialForce;
      }
      else
      {
         // We want to move inward, and we're less likely to hit a
         // joint limit if we move the elbow down
         nullSpaceMotion[2] += radialForce;
      }
   }

   // Head lock

   if ( gv.singularities & HEAD_LOCK )
   {
      // Find out where q1 should point, in order to be able to rotate
      // the end-effector to the right place
      Float desiredQ1;
      if( force[1] >= 0 )
      {
         desiredQ1 = atan2( force[1], force[0] );
      }
      else
      {
         desiredQ1 = atan2( -force[1], -force[0] );
      }

      // Find out how strong the force is in the x-y plane, and scale
      // the q1 torque accordingly.  (No need to spin q1 around if the
      // end-effector is hovering over the base.)
      Float scale = sqrt( force[0] * force[0] + force[1] * force[1] );

      nullSpaceMotion[0] += scale * ( desiredQ1 - q1 );
   }

   // Wrist lock

   if( gv.singularities & WRIST_LOCK )
   {
      // Translate the angular component of force into the x4/y4
      // plane
      PrVector3 x4( R4[0][0], R4[1][0], R4[2][0] );
      PrVector3 y4( R4[0][1], R4[1][1], R4[2][1] );

      Float torqueX4 = force[3] * x4[0] + force[4] * x4[1] + force[5] * x4[2];
      Float torqueY4 = force[3] * y4[0] + force[4] * y4[1] + force[5] * y4[2];

      // Find out where q4 should point, in order to be able to
      // apply the desired angular momentum

      Float desiredQ4;
      if( torqueY4 >= 0 )
      {
         desiredQ4 = atan2( torqueY4, torqueX4 );
      }
      else
      {
         desiredQ4 = atan2( -torqueY4, -torqueX4 );
      }

      // Find out how strong the torque vector is, projected into the
      // x-y plane, and scale the q4 torque accordingly.  (No need to
      // spin q4 around if the finger is going to stay straight.)
      Float scaleFactor = sqrt( torqueX4 * torqueX4 + torqueY4 * torqueY4 );

      nullSpaceMotion[3] += scaleFactor * ( desiredQ4 - q4 );
   }

   // Calculate escapeTorque
   //
   escapeTorque.setSize( 6 );
   for( int ii = 0; ii < 6; ii++ )
   {
      static const Float MIN_KV = 1e-4;
      Float kp  = gv.kp[ii];
      Float kv  = max( gv.kv[ii], MIN_KV );
      Float dqd = nullSpaceMotion[ii] * kp / kv;
      dqd = max( dqd, -gv.dqmax[ii] );
      dqd = min( dqd,  gv.dqmax[ii] );
      escapeTorque[ii] = -kv * ( gv.dq[ii] - dqd );
   }
}

void OpDynamics( const PrVector& fPrime, GlobalVariables& gv )
{
   // No singularities

   if( gv.singularities == 0 )
   {
      gv.tau = gv.Jtranspose * (gv.Lambda * fPrime + gv.mu + gv.p);
      return;
   }

   // Singular configuration

   static PrMatrix nonsingularSelection;
   static PrVector escapeTorque;
   static PrMatrix Ainverse;
   static PrMatrix rJacobian;
   static PrMatrix rJacobianT;
   static PrMatrix rLambdaInverse;
   static PrMatrix rLambda;
   static PrMatrix rJacobianInverseT;
   static PrMatrix identity;
   static PrMatrix nullSpaceT;

   GetNonsingularSelection( nonsingularSelection, gv );
   GetEscapeTorque( fPrime, escapeTorque, gv );
   gv.A.inverseSPD( Ainverse );
   rJacobian = nonsingularSelection * gv.J;
   rJacobian.transpose( rJacobianT );
   rLambdaInverse = rJacobian * Ainverse * rJacobianT;
   rLambdaInverse.inverseSPD( rLambda );
   rJacobianInverseT = rLambda * rJacobian * Ainverse;
   identity.setSize( 6, 6 );
   identity.identity();
   nullSpaceT = identity - rJacobianT * rJacobianInverseT;
   gv.tau = rJacobianT * rLambda * nonsingularSelection * fPrime
         + nullSpaceT * Ainverse * escapeTorque
         + gv.G;
}

/********************************************************
 / VISUAL SERVOING Add-on
*******************************************************/
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
   for( int ii = 0; ii < gv.dof; ii++ )
   {
      gv.qd[ii] = min( max( gv.qd[ii], gv.qmin[ii] ), gv.qmax[ii] );
   }
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
   FindOperationalCubicSpline(gv);
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
	gv.tau = gv.G;
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
   floatControl(gv);  // Remove this line when you implement openControl
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njgotoControl(GlobalVariables& gv) 
{	
   floatControl(gv);  // Remove this line when you implement openControl
}

void jgotoControl(GlobalVariables& gv) 
{
   static const Float MIN_KV = 1e-4;
   PrVector local_dqd = gv.dqd;

   for( int ii = 0; ii < gv.dof; ii++ )
   {
      local_dqd[ii] -= (gv.q[ii] - gv.qd[ii]) * gv.kp[ii] / max(gv.kv[ii], MIN_KV);
      local_dqd[ii] = min(max(local_dqd[ii], -gv.dqmax[ii]), gv.dqmax[ii]);
   }
   PrVector tauPrime = -gv.kv * (gv.dq - local_dqd);
   gv.tau = gv.A * tauPrime + gv.B + gv.G;
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
   PrVector fPrime;
   getSaturatedFprime( fPrime, gv );
   OpDynamics( fPrime, gv );
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) 
{
   PrVector local_xd;
   PrVector local_dxd;
   PrVector local_ddxd;  // ignored
   EvaluateCubicSpline( local_xd, local_dxd, local_ddxd, gv );

   PrVector fPrime = ( -gv.kp * (gv.Einverse * (gv.x-local_xd))
                       - gv.kv * (gv.dx - gv.Einverse * local_dxd) );
   OpDynamics( fPrime, gv );
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

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
