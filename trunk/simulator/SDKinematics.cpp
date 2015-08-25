#include <cmath>
#include <iostream>
#include "SDKinematics.h"
#include "SDConstants.h"

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif

using namespace std;

// define here so the file can be compiled without SDUtils
double SDClip2(double x, double min, double max)
{
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}


// our kinematics use position at CENTER of foot, not tip
bool SDInvKin(int leg, bduVec3f& pos, bduVec3f *qForward, bduVec3f *qBackward)
{
  SDConstants sdc = SDConstants::getInstance();
  double xSign = (leg < 2 ? 1.0 : -1.0);
  double ySign = (leg % 2 == 0 ? 1.0 : -1.0);
  double x,y,z, x2, y2, l1, l2, r, alpha, beta;
  double hipY, kneeZ, footX, footZ, temp;
  bool reachable = true;

  // calculate proper offsets
  hipY = ySign * sdc.mLDHipRyOffsetY;
  kneeZ = sdc.mLDKneeRyOffsetZ;
  footX = xSign * sdc.mLDFootOffsetX;
  footZ = sdc.mLDFootOffsetZ + sdc.mLDFootRadius;

  x = pos.n[0] - sdc.mLDHipRxOffsetX * xSign;
  y = pos.n[1] - sdc.mLDHipRxOffsetY * ySign;
  z = pos.n[2];

  // hairy inverse trig
  temp = hipY / sqrt(y*y + z*z);
  if (temp < -1.0 || temp > 1.0) reachable = false;
  qForward->n[0] = qBackward->n[0] = atan2(z, y) +
    acos(SDClip2(temp, -1.0, 1.0));
  
  x2 = x;
  y2 = sqrt(pow(y - hipY * cos(qForward->n[0]), 2) +
	    pow(z - hipY * sin(qForward->n[0]), 2));
  l1 = fabs(kneeZ);
  l2 = sqrt(footX*footX + footZ*footZ);
  r = sqrt(x2*x2 + y2*y2);

  temp = (r*r + l1*l1 - l2*l2) / (2 * l1 * r);
  if (temp < -1.0 || temp > 1.0) reachable = false;
  beta = acos(SDClip2(temp, -1.0, 1.0));
  qForward->n[1] = M_PI/2 - atan2(y2,-x2) - beta;
  qBackward->n[1] = M_PI/2 - atan2(y2,-x2) + beta;

  temp = (l1*l1 + l2*l2 - r*r) / (2 * l1 * l2);
  if (temp < -1.0 || temp > 1.0) reachable = false;
  alpha = acos(SDClip2(temp, -1.0, 1.0));
  qForward->n[2] = atan2(footX, -footZ) - (alpha - M_PI);
  qBackward->n[2] = atan2(footX, -footZ) + (alpha - M_PI);

  return reachable;
}


// forward kinematics to foot center
void SDFwdKin(int leg, const bduVec3f& angles, bduVec3f *pos)
{
  SDConstants sdc = SDConstants::getInstance();
  double xSign = (leg < 2 ? 1.0 : -1.0);
  double ySign = (leg % 2 == 0 ? 1.0 : -1.0);
  double thetaHx, thetaHy, thetaK;
  double hipY, kneeZ, footX, footZ;

  // calculate proper offsets
  hipY = ySign * sdc.mLDHipRyOffsetY;
  kneeZ = sdc.mLDKneeRyOffsetZ;
  footX = xSign * sdc.mLDFootOffsetX;
  footZ = sdc.mLDFootOffsetZ + sdc.mLDFootRadius;

  thetaHx = -angles.n[0];
  thetaHy = -angles.n[1];
  thetaK = -angles.n[2];

  pos->n[0] = sdc.mLDHipRxOffsetX * xSign + footX * cos(thetaHy + thetaK) - 
    footZ * sin(thetaHy + thetaK) - kneeZ * sin(thetaHy);

  pos->n[1] = sdc.mLDHipRxOffsetY * ySign + sin(thetaHx) *
    (footX * sin(thetaHy + thetaK) + footZ * cos(thetaHy + thetaK) +
     kneeZ * cos(thetaHy)) + hipY * cos(thetaHx);

  pos->n[2] = cos(thetaHx) *
    (footX * sin(thetaHy + thetaK) + footZ * cos(thetaHy + thetaK) +
     kneeZ * cos(thetaHy)) - hipY * sin(thetaHx);
}

// forward kinematics to all joints
void SDFullFwdKin(int leg, const bduVec3f& angles, bduVec3f* hipRx,
		  bduVec3f *hipRy, bduVec3f *knee, bduVec3f *foot,
		  bduVec3f *hipRxAxis, bduVec3f *hipRyAxis, bduVec3f *kneeAxis)
{
  SDConstants sdc = SDConstants::getInstance();
  double xSign = (leg < 2 ? 1.0 : -1.0);
  double ySign = (leg % 2 == 0 ? 1.0 : -1.0);
  double thetaHx, thetaHy, thetaK;
  double hipY, kneeZ, footX, footZ, hipRxX, hipRxY;

  // calculate proper offsets
  hipRxX = xSign * sdc.mLDHipRxOffsetX;
  hipRxY = ySign * sdc.mLDHipRxOffsetY;
  hipY = ySign * sdc.mLDHipRyOffsetY;
  kneeZ = sdc.mLDKneeRyOffsetZ;
  footX = xSign * sdc.mLDFootOffsetX;
  footZ = sdc.mLDFootOffsetZ + sdc.mLDFootRadius;

  thetaHx = -angles.n[0];
  thetaHy = -angles.n[1];
  thetaK = -angles.n[2];

  if (hipRx != 0) {
    hipRx->n[0] = hipRxX;
    hipRx->n[1] = hipRxY;
    hipRx->n[2] = 0.0;
  }

  if (hipRx != 0) {
    hipRy->n[0] = hipRxX;
    hipRy->n[1] = hipRxY + cos(thetaHx) * hipY;
    hipRy->n[2] = -sin(thetaHx) * hipY;
  }

  if (knee != 0) {
    knee->n[0] = hipRxX - sin(thetaHy) * kneeZ;
    knee->n[1] = hipRxY + sin(thetaHx) * cos(thetaHy) * kneeZ +
      cos(thetaHx) * hipY;
    knee->n[2] = cos(thetaHx) * cos(thetaHy) * kneeZ - sin(thetaHx) * hipY;
  }

  if (foot != 0) {
    SDFwdKin(leg, angles, foot);
  }

  if (hipRxAxis != 0) {
    hipRxAxis->n[0] = 1.0;
    hipRxAxis->n[1] = 0.0;
    hipRxAxis->n[2] = 0.0;
  }

  if (hipRyAxis != 0) {
    hipRyAxis->n[0] = 0.0;
    hipRyAxis->n[1] = cos(thetaHx);
    hipRyAxis->n[2] = -sin(thetaHx);
  }

  if (kneeAxis != 0) {
    kneeAxis->n[0] = 0.0;
    kneeAxis->n[1] = cos(thetaHx);
    kneeAxis->n[2] = -sin(thetaHx);
  }
}
  

// compute foot location along a line given knee angle
bool SDFwdInvKin(int leg, double kneeAngle, const bduVec3f &p1,
		 const bduVec3f &p2, bduVec3f *angles, double *t)
{
  SDConstants sdc = SDConstants::getInstance();
  double xSign = (leg < 2 ? 1.0 : -1.0);
  double ySign = (leg % 2 == 0 ? 1.0 : -1.0);
  double a, b, c, l, d;
  double hipY, kneeZ, footX, footZ;
  double forwardDiff, backwardDiff;
  bool lineIntersects, ikReachable;
  bduVec3f pos, jointsF, jointsB;

  hipY = ySign * sdc.mLDHipRyOffsetY;
  kneeZ = sdc.mLDKneeRyOffsetZ;
  footX = xSign * sdc.mLDFootOffsetX;
  footZ = sdc.mLDFootOffsetZ + sdc.mLDFootRadius;

  // determine the radius of the sphere of reachability
  l = SQR(hipY) + SQR(kneeZ) + SQR(footX) + SQR(footZ) -
    2 * fabs(kneeZ) * sqrt(SQR(footX) + SQR(footZ)) *
    cos(M_PI + kneeAngle - atan2(footX, -footZ));
  

  // find the intersections of the line and sphere
  a = SQR(p2.n[0]-p1.n[0]) + SQR(p2.n[1]-p1.n[1]) + SQR(p2.n[2]-p1.n[2]);
  b = 2 * (p1.n[0] - sdc.mLDHipRxOffsetX * xSign) * (p2.n[0] - p1.n[0])
    + 2 * (p1.n[1] - sdc.mLDHipRxOffsetY * ySign) * (p2.n[1] - p1.n[1])
    + 2 * p1.n[2] * (p2.n[2] - p1.n[2]);
  c = SQR(p1.n[0] - sdc.mLDHipRxOffsetX * xSign) +
    SQR(p1.n[1] - sdc.mLDHipRxOffsetY * ySign) + SQR(p1.n[2]) - l;
  d = b*b - 4*a*c;

  if (d < 0) {
    lineIntersects = false;
    *t = -b / (2*a);
  } else {
    lineIntersects = true;
    *t = (-b + sqrt(d)) / (2*a);
  }

  // find complete inverse kinematic solution
  pos.n[0] = p1.n[0] + *t * (p2.n[0] - p1.n[0]);
  pos.n[1] = p1.n[1] + *t * (p2.n[1] - p1.n[1]);
  pos.n[2] = p1.n[2] + *t * (p2.n[2] - p1.n[2]);
  ikReachable = SDInvKin(leg, pos, &jointsF, &jointsB);
  forwardDiff = fabs(jointsF.n[2] - kneeAngle);
  backwardDiff = fabs(jointsB.n[2] - kneeAngle);

  if(forwardDiff <= backwardDiff) {
    angles->n[0] = jointsF.n[0];
    angles->n[1] = jointsF.n[1];
    angles->n[2] = jointsF.n[2];
  } else {
    angles->n[0] = jointsB.n[0];
    angles->n[1] = jointsB.n[1];
    angles->n[2] = jointsB.n[2];
  }

  return (lineIntersects && ikReachable);
}
