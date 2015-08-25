#ifndef __SD_UTILS_H__
#define __SD_UTILS_H__


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "SDLittleDog.h"

#ifndef MAX
#define MAX(a,b) (a > b ? a : b)
#endif

#ifndef MIN
#define MIN(a,b) (a < b ? a : b)
#endif

#ifndef SIGN
#define SIGN(a) (a >= 0 ? 1 : -1)
#endif

#ifndef ABS
#define ABS(a) (a >= 0 ? a : -a)
#endif

#ifndef SQR
#define SQR(X) (x*x)
#endif


// error handling functions
void SDHandleErr(const SDLittleDog &dog, const char* function,
		   LD_ERROR_CODE result, bool exitOnErr);

void SDPrintErr(const char* error);


// drawing routines

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#define dsDrawLine dsDrawLineD
#endif

void SDDrawGeom(dGeomID geom);
void SDSetDrawTerrain(bool drawTerrain);
bool SDGetDrawTerrain();
void SDDrawPoint(const dVector3 p, double radius);
void SDDrawLine(const dVector3 p1, const dVector3 p2);

// distance routines
double SDDistanceSquared(const bduVec3f& v1, const bduVec3f& v2, int len);
double SDDistance(const bduVec3f& v1, const bduVec3f& v2, int len);

// goal position routine
void SDGetGoalPosition(bduVec3f *goal);

// vector routines
double SDNorm(const bduVec3f& vec, int len);
void SDNormalizeVector(bduVec3f& vec, int len);

// angle routines
double SDNormalizeAngle(double theta);
double SDNormalizeAngle2(double theta);
double SDInterpolateAngles(double theta1, double theta2, double t);
double SDAngleDifference(double theta1, double theta2);
double SDSignPower(double x, double p);
double SDClip(double x, double min, double max);
void SDRotateVector2D(bduVec3f *v, double angle, int i, int j);
void SDRotate2D(double *x, double *y, double angle);

// euler - quaternion conversion
void SDQuaternionToEuler(const bduVec4f& quatOri, bduVec3f *eulerOri);
void SDEulerToQuaternion(const bduVec3f& eulerOri, bduVec4f *quatOri);

// world - local conversion
void SDWorldToLocal(const bduVec3f& pos, const bduVec3f& ori,
		    const bduVec3f& world, bduVec3f* local);
void SDLocalToWorld(const bduVec3f& pos, const bduVec3f& ori,
		    const bduVec3f& local, bduVec3f* world);

// random functions
void SDSeedRandom(int seed);
double SDRandomNormal(double mean, double sig);
double SDRandomUniform(double min, double max);


#endif
     
		   
