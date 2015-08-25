#include "SDUtils.h"
#include "SDTerrain.h"
#include <iostream>
#include <bduLog.h>
#include <cmath>
#include <cstdlib>

using namespace std;

#define D_USE_BDU_LOG

// handle a LD_ERROR_CODE result
void SDHandleErr(const SDLittleDog &dog, const char* function,
		 LD_ERROR_CODE result, bool exitOnError)
{
  if (result == LD_OKAY) return;

  /*
#ifdef D_USE_BDU_LOG
  bdu_log_printf(BDU_LOG_LEVEL_WARN,
		 "Error in function '%s()': \n   %s.\n",
		 function, dog.getErrorCodeString(result));
#else
  cout << "Error in function '" << function << "()': " << endl;
  cout << "  " << dog.getErrorCodeString(result) << endl;
#endif

  if (exitOnError) exit(1);
  */
}

// print a static error message
void SDPrintErr(const char* error)
{
#ifdef D_USE_BDU_LOG
  bdu_log_printf(BDU_LOG_LEVEL_WARN, "%s\n", error);
#else
  cout << error << endl;
#endif
}


// get/set whether to draw the terrain
bool gDrawTerrain = false;
void SDSetDrawTerrain(bool drawTerrain)
{
  gDrawTerrain = drawTerrain;
}
bool SDGetDrawTerrain()
{
  return gDrawTerrain;
}


// draw an ode geom
void SDDrawGeom(dGeomID geom)
{
  int type = dGeomGetClass(geom);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (geom,sides);
    dsDrawBox(dGeomGetPosition(geom), dGeomGetRotation(geom), sides);
  } else if (type == dCCylinderClass) {
    dReal radius, length;
    dGeomCCylinderGetParams(geom, &radius, &length);
    dsDrawCappedCylinder(dGeomGetPosition(geom), dGeomGetRotation(geom),
			 length, radius);
  } else if (type == dTriMeshClass) {
    if (gDrawTerrain) {
      SDTerrain *terrain = (SDTerrain*)dGeomGetData(geom);
      terrain->dsDraw();
    }
  }
}

// draw a point as a small sphere
void SDDrawPoint(const dVector3 p, double radius)
{
  dMatrix3 noRotation = {1, 0, 0, 0,
			 0, 1, 0, 0,
			 0, 0, 1, 0};
  
  dsDrawSphere(p, noRotation, radius);
}

// draw a line
void SDDrawLine(const dVector3 p1, const dVector3 p2)
{
  dsDrawLineD(p1,p2);
}


// returns the squared distance between two vectors 
double SDDistanceSquared(const bduVec3f& v1, const bduVec3f& v2, int len)
{
	double result = 0.0;
	for ( int i = 0; i < len; i++ )
	{
		double diff = v2.n[i] - v1.n[i];
		result += diff * diff;
	}
	return result;
}

// returns the distance between two vectors
double SDDistance(const bduVec3f& v1, const bduVec3f& v2, int len)
{
	return sqrt(SDDistanceSquared(v1, v2, len));
}

// return the position of the goal
void SDGetGoalPosition(bduVec3f *goal)
{
  char path[500];
  ifstream fin;
  strcpy(path, getenv("LITTLEDOG"));
  strcat(path, "/test/goal.txt");

  fin.open(path);
  if (fin.good()) {
    fin >> goal->n[0] >> goal->n[1];
  } else {
    goal->n[0] = 1.0;
    goal->n[1] = 0.0;
  }
}

// returns the norm of a vector
double SDNorm(const bduVec3f& vec, int len)
{
	double result = 0.0;
	for ( int i = 0; i < len; i++ )
	{
		result += vec.n[i] * vec.n[i];
	}
	result = sqrt(result);
	return result;
}

// normalizes a vector such that its values sum to 1
void SDNormalizeVector(bduVec3f& vec, int len)
{
	double n = SDNorm(vec, len);
	n = (n == 0) ? 1 : n;	

	for ( int i = 0; i < len; i++ )
	{
		vec.n[i] /= n;
	}
}

// normalize an angle to [0, 2*PI]
double SDNormalizeAngle(double theta)
{
  while (theta >= 2*M_PI) theta -= 2*M_PI;
  while (theta < 0) theta += 2*M_PI;
  return theta;
}


// normalize an angle to [-PI, PI]
double SDNormalizeAngle2(double theta)
{
  while (theta >= M_PI) theta -= 2*M_PI;
  while (theta < -M_PI) theta += 2*M_PI;
  return theta;
}


// interpolate between two angles
double SDInterpolateAngles(double theta1, double theta2, double t)
{
  theta1 = SDNormalizeAngle(theta1);
  theta2 = SDNormalizeAngle(theta2);
  
  // interpolate between the angles
  if (fabs(theta1 - theta2) < M_PI) {
    return (1-t)*theta1 + t*theta2;
  } else {
    if (theta1 < theta2) {
      theta2 -= 2*M_PI;
    } else {
      theta1 -= 2*M_PI;
    }
    return SDNormalizeAngle((1-t)*theta1 + t*theta2);
  }
}


// return the absolute difference between two angles
double SDAngleDifference(double theta1, double theta2)
{
  theta1 = SDNormalizeAngle(theta1);
  theta2 = SDNormalizeAngle(theta2);
  
  // interpolate between the angles
  if (fabs(theta1 - theta2) < M_PI) {
    return fabs(theta1 - theta2);
  } else {
    if (theta1 < theta2) {
      theta2 -= 2*M_PI;
    } else {
      theta1 -= 2*M_PI;
    }
    return fabs(theta1 - theta2);
  }
}

// return a number raised to some power with it's original sign
double SDSignPower(double x, double p)
{
  return pow(fabs(x), p) * (x > 0 ? 1: -1);
}

// clip a number to within (-max, max)
double SDClip(double x, double min, double max)
{
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}

// rotate two components of a vector by angle
void SDRotateVector2D(bduVec3f *v, double angle, int i, int j)
{
  float x, y;

  x = v->n[i];
  y = v->n[j];

  v->n[i] = x * cos(angle) - y * sin(angle);
  v->n[j] = x * sin(angle) + y * cos(angle);
}

// rotate a point by an angle
void SDRotate2D(double *x, double *y, double angle)
{
  double tempX = *x;
  double tempY = *y;

  *x = tempX * cos(angle) - tempY * sin(angle);
  *y = tempX * sin(angle) + tempY * cos(angle);
}


// convert a quaternion to euler angles
void SDQuaternionToEuler(const bduVec4f &qOri, bduVec3f *eOri)
{
  eOri->n[0] = atan2(2*(qOri.n[1] * qOri.n[2] + qOri.n[3] * qOri.n[0]),
			 pow(qOri.n[3], 2) - pow(qOri.n[0], 2) -
			 pow(qOri.n[1], 2) + pow(qOri.n[2], 2));

  eOri->n[1] = asin(-2*(qOri.n[0] * qOri.n[2] - qOri.n[3] * qOri.n[1]));

  eOri->n[2] = atan2(2*(qOri.n[0] * qOri.n[1] + qOri.n[3] * qOri.n[2]),
			 pow(qOri.n[3], 2) + pow(qOri.n[0], 2) -
			 pow(qOri.n[1], 2) - pow(qOri.n[2], 2));
}

// converts euler angles to quaternion
void SDEulerToQuaternion(const bduVec3f& eulerOri, bduVec4f *quatOri)
{
  // This math is from:
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // Translation between their terminology and ours... as near as I can tell
  //
  // q0 -> q.n[3]
  // q1 -> q.n[0]
  // q2 -> q.n[1]
  // q3 -> q.n[2]
  
  double c1 = cos(eulerOri.n[0]/2);
  double c2 = cos(eulerOri.n[1]/2);
  double c3 = cos(eulerOri.n[2]/2);
  
  double s1 = sin(eulerOri.n[0]/2);
  double s2 = sin(eulerOri.n[1]/2);
  double s3 = sin(eulerOri.n[2]/2);
  
  quatOri->n[3] = c1*c2*c3 + s1*s2*s3; 
  quatOri->n[0] = s1*c2*c3 - c1*s2*s3;
  quatOri->n[1] = c1*s2*c3 + s1*c2*s3;
  quatOri->n[2] = c1*c2*s3 - s1*s2*c3;
}

// converts world coordinates to local coordinates using euler angles
void SDWorldToLocal(const bduVec3f& pos, const bduVec3f& ori,
		    const bduVec3f& world, bduVec3f* local)
{
  double c1 = cos(ori.n[0]);
  double c2 = cos(ori.n[1]);
  double c3 = cos(ori.n[2]);
  
  double s1 = sin(ori.n[0]);
  double s2 = sin(ori.n[1]);
  double s3 = sin(ori.n[2]);
  
  double r[3][3];
  r[0][0] = c2*c3;
  r[0][1] = c2*s3;
  r[0][2] = -s2;
  r[1][0] = -c1*s3 + s1*s2*c3;
  r[1][1] = c1*c3 + s1*s2*s3;
  r[1][2] = s1*c2;
  r[2][0] = s1*s3 + c1*s2*c3;
  r[2][1] = -s1*c3 + c1*s2*s3;
  r[2][2] = c1*c2;
  
  bduVec3f temp;
  for ( int i = 0; i < 3; i++ ) {
    temp.n[i] = world.n[i] - pos.n[i];
  }
  
  for ( int i = 0; i < 3; i++ )	{
    local->n[i] = r[i][0]*temp.n[0] + r[i][1]*temp.n[1] + r[i][2]*temp.n[2];
  }
}

void SDLocalToWorld(const bduVec3f& pos, const bduVec3f& ori,
		    const bduVec3f& local, bduVec3f* world)
{
  double c1 = cos(ori.n[0]);
  double c2 = cos(ori.n[1]);
  double c3 = cos(ori.n[2]);
  
  double s1 = sin(ori.n[0]);
  double s2 = sin(ori.n[1]);
  double s3 = sin(ori.n[2]);
  
  double r[3][3];
  r[0][0] = c2*c3;
  r[0][1] = -c1*s3 + s1*s2*c3;
  r[0][2] = s1*s3 + c1*s2*c3;
  r[1][0] = c2*s3;
  r[1][1] = c1*c3 + s1*s2*s3;
  r[1][2] = -s1*c3 + c1*s2*s3;
  r[2][0] = -s2;
  r[2][1] = s1*c2;
  r[2][2] = c1*c2;
  
  for ( int i = 0; i < 3; i++ ) {
    world->n[i] = r[i][0]*local.n[0] + r[i][1]*local.n[1] +
      r[i][2]*local.n[2] + pos.n[i];
  }
}

// seed the random number generator
void SDSeedRandom(int seed)
{
  srand(seed);
}

// return a normally distrubted random number
double SDRandomNormal(double mean, double sig)
{
  // logic taken from GSL code
  double x, y, r;
  do {
    x = SDRandomUniform(-1, 1);
    y = SDRandomUniform(-1, 1);
    r = x * x + y * y;
  } while (r > 1.0 || r == 0.0);
  return sig * y * sqrt (-2.0 * log(r) / r) + mean;
}

// return a uniformly distributed random number
double SDRandomUniform(double min, double max)
{
  return ((double)rand() / RAND_MAX) * (max - min) + min;
}



