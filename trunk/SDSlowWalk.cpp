#include "SDSlowWalk.h"
#include "SDKinematics.h"
#include "SDUtils.h"
#include "SDConstants.h"
#include "SDVectorOp.h"
#include <fstream>
#include <iostream>

using namespace std;

// reset the controller
bool SDSlowWalk::reset()
{
  ifstream fin;

  fin.open("params/slowwalk.params");
  if (fin.good()) {
    fin >> mBodySpeed >> mFootSpeed;
    fin >> mFootHeight >> mStepHeight >> mCGMargin;
    fin >> mPauseTime >> mRepositionTime;
  }

  mLittleDog->updateDogState();
  mLittleDog->getDogState(&mCurrentState);

  // initialize foot positions
  for (int i = 0; i < NUM_LEGS; i++) {
    SDFwdKin(i, mCurrentState.jointAngles[i], &mFootPos[i]);
  }
  beginShiftCG();

  return true;
}

// main control loop, get angles
void SDSlowWalk::getAngles(bduVec3f *angles)
{
  mLittleDog->updateDogState();
  mLittleDog->getDogState(&mCurrentState);

  bduVec3f footPos[4];
  double t = mCurrentState.timestamp - mStartTime;

  if (mControllerMode == SD_SLOW_WALK_SHIFT_CG) {
    // interpolate between mFeetPos and mNewFeetPos
    for (int i = 0; i < NUM_LEGS; i++) {
      SD_VEC_INTERP(footPos[i], mFootPos[i], mNextFootPos[i],
		    SDClip(t / mShiftCGTime, 0.0, 1.0));
    }

    // if we're done moving and pausing, start taking the step
    if (t > mShiftCGTime + mPauseTime) {
      beginStep();
    }
    
  } else if (mControllerMode == SD_SLOW_WALK_STEP) {

    // position the moving foot
    if (t < mWaypointTime[0]) {
      SD_VEC_INTERP(footPos[mMovingFoot], mNextFootPos[mMovingFoot],
		    mStepWaypoints[0], t / mWaypointTime[0]);
    } else if (t < mWaypointTime[0] + mWaypointTime[1]) {
      SD_VEC_INTERP(footPos[mMovingFoot], mStepWaypoints[0],
		    mStepWaypoints[1],
		    (t - mWaypointTime[0]) / mWaypointTime[1]);
    } else if (t < mWaypointTime[0] + mWaypointTime[1] + mWaypointTime[2]) {
      SD_VEC_INTERP(footPos[mMovingFoot], mStepWaypoints[1],
		    mStepWaypoints[2],
		    (t-mWaypointTime[0]-mWaypointTime[1]) / mWaypointTime[2]);
    } else {
      SD_VEC_COPY(footPos[mMovingFoot], mStepWaypoints[2]);
    }

    // position the supporting feet
    for (int i = 0; i < NUM_LEGS; i++) {
      if (i != mMovingFoot) {
	SD_VEC_COPY(footPos[i], mNextFootPos[i]);
      }
    }

    if (t > mStepTime + mPauseTime) {
      beginReposition();
    }

  } else if (mControllerMode == SD_SLOW_WALK_REPOSITION) {

    // move the feet to their current position (for no jitters)
    for (int i = 0; i < NUM_LEGS; i++) {
      if (i == mMovingFoot) {
	SD_VEC_INTERP(footPos[i], mStepWaypoints[2], mFootPos[i],
		      t / mRepositionTime);
      } else {
	SD_VEC_INTERP(footPos[i], mNextFootPos[i], mFootPos[i],
		      t / mRepositionTime);
      }
    }

    if (t > mRepositionTime) {
      //beginShiftCG();
      mLittleDog->stopRobot();
    }
  }

  // set the angles
  bduVec3f junk;
  /*
  if (!SDInvKin(LittleDog::FL, footPos[0], &junk, &angles[LittleDog::FL]))
    cout << mCurrentState.timestamp << " Front Left Unreachable" << endl;
  if (!SDInvKin(LittleDog::HR, footPos[3], &angles[LittleDog::HR], &junk))
    cout << mCurrentState.timestamp << " Back Right Unreachable" << endl;
  if (!SDInvKin(LittleDog::FR, footPos[1], &junk, &angles[LittleDog::FR]))
    cout << mCurrentState.timestamp << " Front Right Unreachable" << endl;
  if (!SDInvKin(LittleDog::HL, footPos[2], &angles[LittleDog::HL], &junk))
    cout << mCurrentState.timestamp << " Back Left Unreachable" << endl;
  */
  SDInvKin(LittleDog::FL, footPos[0], &junk, &angles[LittleDog::FL]);
  SDInvKin(LittleDog::HR, footPos[3], &angles[LittleDog::HR], &junk);
  SDInvKin(LittleDog::FR, footPos[1], &junk, &angles[LittleDog::FR]);
  SDInvKin(LittleDog::HL, footPos[2], &angles[LittleDog::HL], &junk);
}

// draw controller-related information
void SDSlowWalk::dsDraw()
{
  
}

// set the next footstep
void SDSlowWalk::setStepPosition(int movingFoot, double x, double y)
{
  SD_VEC_SET(mGlobalStepPos, x, y, 0);
  mGlobalStepPos.n[2] =  mLittleDog->getSimulator()->getPointHeight(x, y);
  mMovingFoot = movingFoot;
}


// set up the CG shift
void SDSlowWalk::beginShiftCG()
{
  // get the next moving foot and the position to move it to

  // find the height of each foot and the next foot position
  bduVec3f globalFootPos[NUM_LEGS];
  bduVec3f relativeFootPos[NUM_LEGS];
  bduVec3f relativeStepPos, localStepPos;
  double footHeight[NUM_LEGS];
  double nextStepHeight;
  bduVec3f pos, ori;

  nextStepHeight = mGlobalStepPos.n[2];
  SD_VEC_SET(pos, 0, 0, 0);
  SD_VEC_SET(ori, mCurrentState.ori.n[0], mCurrentState.ori.n[1], 0);
  SDWorldToLocal(mCurrentState.pos, mCurrentState.ori,
		 mGlobalStepPos, &localStepPos);
  SDLocalToWorld(pos, ori, localStepPos, &relativeStepPos);
  for (int i = 0; i < NUM_LEGS; i++) {
    SDLocalToWorld(mCurrentState.pos, mCurrentState.ori, mFootPos[i],
		   &globalFootPos[i]);
    SDLocalToWorld(pos, ori, mFootPos[i], &relativeFootPos[i]);
    footHeight[i] =
      mLittleDog->getSimulator()->getPointHeight(globalFootPos[i].n[0],
						 globalFootPos[i].n[1]);
    
  }
  mGlobalStepPos.n[2] += SDConstants::getInstance().mLDFootRadius;

  // find the height of the front and back of the dog
  double frontHeight, backHeight;
  if (mMovingFoot == 0 || mMovingFoot == 1) {
    frontHeight = MIN(MIN(footHeight[0], footHeight[1]), nextStepHeight);
  } else {
    frontHeight = MIN(footHeight[0], footHeight[1]);
  }
  if (mMovingFoot == 2 || mMovingFoot == 3) {
    backHeight = MIN(MIN(footHeight[2], footHeight[3]), nextStepHeight);
  } else {
    backHeight = MIN(footHeight[2], footHeight[3]);
  }

  // determine the (local) x,y positions of the next foot positions
  bduVec3f supportTriangle[3];
  bduVec3f feetCenter, center;
  
  getSupportTriangle(relativeFootPos, supportTriangle);
  getFeetCenter(relativeFootPos, &feetCenter);
  getMarginCenter(feetCenter, supportTriangle, &center);

  for (int i = 0; i < NUM_LEGS; i++) {
    mNextFootPos[i].n[0] = relativeFootPos[i].n[0] - center.n[0];
    mNextFootPos[i].n[1] = relativeFootPos[i].n[1] - center.n[1];
    mNextFootPos[i].n[2] = footHeight[i] + mFootHeight;
  }

  // find the desired pitch of the dog and rotate the legs accordingly
  double pitch = atan2(frontHeight - backHeight,
		       SDConstants::getInstance().mLDTrunkSizeX);
  double extraHeight = (frontHeight + backHeight) / 2.0;
  for (int i = 0; i < NUM_LEGS; i++) {
    mNextFootPos[i].n[2] -= SDConstants::getInstance().mLDTrunkCOMZ;
    SDRotateVector2D(&mNextFootPos[i], -pitch, 0, 2);
    mNextFootPos[i].n[2] -= extraHeight;
    mNextFootPos[i].n[2] += SDConstants::getInstance().mLDTrunkCOMZ;
  }

  
  // find the desired yaw for the robot
  bduVec3f leftFeetVec, rightFeetVec;
  double yaw;
  if (mMovingFoot == 0 || mMovingFoot == 2) {
    if (mMovingFoot == 0) {
      SD_VEC_SUB(leftFeetVec, relativeStepPos, relativeFootPos[2]);
    } else {
      SD_VEC_SUB(leftFeetVec, relativeFootPos[0], relativeStepPos);
    }
    SD_VEC_SUB(rightFeetVec, relativeFootPos[1], relativeFootPos[3]);
  } else {
    if (mMovingFoot == 1) {
      SD_VEC_SUB(rightFeetVec, relativeStepPos, relativeFootPos[3]);
    } else {
      SD_VEC_SUB(rightFeetVec, relativeFootPos[1], relativeStepPos);
    }
    SD_VEC_SUB(leftFeetVec, relativeFootPos[0], relativeFootPos[2]);
  }
  yaw = SDInterpolateAngles(atan2(leftFeetVec.n[1], leftFeetVec.n[0]),
			    atan2(rightFeetVec.n[1], rightFeetVec.n[0]), 0.5);
  for (int i = 0; i < NUM_LEGS; i++) {
    SDRotateVector2D(&mNextFootPos[i], -yaw, 0, 1);
  }

  // find the time necessary to shift the CG
  mShiftCGTime = (SD_VEC_LENGTH(center) + M_PI * fabs(SDNormalizeAngle2(yaw)) *
		  SDConstants::getInstance().mLDTrunkSizeX / 2 +
		  M_PI * SDAngleDifference(pitch, mCurrentState.ori.n[2]) *
		  SDConstants::getInstance().mLDTrunkSizeX / 2) / mBodySpeed;
  mStartTime = mCurrentState.timestamp;
  mControllerMode = SD_SLOW_WALK_SHIFT_CG;
}



// begin taking a step
void SDSlowWalk::beginStep()
{
  // get the global position of the moving foot
  bduVec3f globalMovingFootPos;
  SDLocalToWorld(mCurrentState.pos, mCurrentState.ori,
		 mNextFootPos[mMovingFoot], &globalMovingFootPos);
  globalMovingFootPos.n[2] = 
    mLittleDog->getSimulator()->getPointHeight(globalMovingFootPos.n[0],
					       globalMovingFootPos.n[1]);
  

  // find the height of the step and set up the global step waypoints
  bduVec3f globalStepWaypoints[2];
  double stepHeight;
  stepHeight = MAX(globalMovingFootPos.n[2],mGlobalStepPos.n[2]) + mStepHeight;
  SD_VEC_COPY(globalStepWaypoints[0], globalMovingFootPos);
  globalStepWaypoints[0].n[2] = stepHeight;
  SD_VEC_COPY(globalStepWaypoints[1], mGlobalStepPos);
  globalStepWaypoints[1].n[2] = stepHeight;

  // convert to local waypoints
  SDWorldToLocal(mCurrentState.pos, mCurrentState.ori,
		 globalStepWaypoints[0], &mStepWaypoints[0]);
  SDWorldToLocal(mCurrentState.pos, mCurrentState.ori,
		 globalStepWaypoints[1], &mStepWaypoints[1]);
  SDWorldToLocal(mCurrentState.pos, mCurrentState.ori,
		 mGlobalStepPos, &mStepWaypoints[2]);

  // determine the (local) x,y positions of the next foot positions
  bduVec3f currentFeet[4], globalFootPos[4];
  bduVec3f feetCenter, center;

  for (int i = 0; i < NUM_LEGS; i++) {
    SDFwdKin(i, mCurrentState.jointAngles[i], &currentFeet[i]);
    SDLocalToWorld(mCurrentState.pos, mCurrentState.ori, currentFeet[i],
		   &globalFootPos[i]);
  }
  
  // determine the distances and times for the step
  double d1, d2, d3;
  d1 = SD_VEC_DIST(mNextFootPos[mMovingFoot], mStepWaypoints[0]);
  d2 = SD_VEC_DIST(mStepWaypoints[0], mStepWaypoints[1]);
  d3 = SD_VEC_DIST(mStepWaypoints[1], mStepWaypoints[2]);

  mStepTime = (d1 + d2 + d3) / mFootSpeed;
  mWaypointTime[0] = d1 / mFootSpeed;
  mWaypointTime[1] = d2 / mFootSpeed;
  mWaypointTime[2] = d3 / mFootSpeed;
  mStartTime = mCurrentState.timestamp;
  mControllerMode = SD_SLOW_WALK_STEP;
}


// begin repositioning the robot
void SDSlowWalk::beginReposition()
{
  for (int i = 0; i < NUM_LEGS; i++) {
    SDFwdKin(i, mCurrentState.jointAngles[i], &mFootPos[i]);
  }
  mStartTime = mCurrentState.timestamp;
  mControllerMode = SD_SLOW_WALK_REPOSITION;
}

// get the next footstep
void SDSlowWalk::getNextFootstep(int *movingFoot, bduVec3f *globalStepPos)
{
  static ifstream fin("footsteps.txt");
  int temp;

  fin >> temp >> globalStepPos->n[0] >> globalStepPos->n[1];
  globalStepPos->n[2] =
    mLittleDog->getSimulator()->getPointHeight(globalStepPos->n[0],
					       globalStepPos->n[1]);
  *movingFoot = temp;
}

// find the center (average) of the feet
void SDSlowWalk::getFeetCenter(bduVec3f *feet, bduVec3f *center)
{
  SD_VEC_SET(*center, 0.0, 0.0, 0.0);
  for (int i = 0; i < NUM_LEGS; i++) {
    center->n[0] += feet[i].n[0] / NUM_LEGS;
    center->n[1] += feet[i].n[1] / NUM_LEGS;
  }
}

// determine the supporting triangle given the moving foot
void SDSlowWalk::getSupportTriangle(bduVec3f *feet, bduVec3f *tri)
{
  bduVec3f fullTri[3];
  switch (mMovingFoot) {
  case LittleDog::FL:
    setGroundTriangle(fullTri, feet[2], feet[1], feet[3]); break;
  case LittleDog::FR:
    setGroundTriangle(fullTri, feet[0], feet[3], feet[2]); break;
  case LittleDog::HL:
    setGroundTriangle(fullTri, feet[3], feet[0], feet[1]); break;
  case LittleDog::HR:
    setGroundTriangle(fullTri, feet[1], feet[2], feet[0]); break;
  }
  insetTriangle(fullTri, tri, mCGMargin);
}

// determine the point closest closest to supporting triangle
void SDSlowWalk::getMarginCenter(const bduVec3f &feetCenter,
				const bduVec3f *supportTri, bduVec3f *p)
{
  bduVec3f triDir, triDirPerp, centerPerp;

  SD_VEC_SUB(triDir, supportTri[0], supportTri[1]);
  SD_VEC_SET(triDirPerp, triDir.n[1], -triDir.n[0], 0);
  SD_VEC_ADD(centerPerp, feetCenter, triDirPerp);
  intersectLines(supportTri[0], supportTri[1], feetCenter, centerPerp, p);
}


// calculate the incenter of a triangle
void SDSlowWalk::getTriangleIncenter(const bduVec3f* p, bduVec3f *incenter)
{
  double d01, d12, d20, dTotal;
  d01 = sqrt(pow(p[0].n[0] - p[1].n[0], 2) + pow(p[0].n[1] - p[1].n[1], 2));
  d12 = sqrt(pow(p[1].n[0] - p[2].n[0], 2) + pow(p[1].n[1] - p[2].n[1], 2));
  d20 = sqrt(pow(p[2].n[0] - p[0].n[0], 2) + pow(p[2].n[1] - p[0].n[1], 2));

  dTotal = d01 + d12 + d20;
  incenter->n[0] = (d12*p[0].n[0] + d20*p[1].n[0] + d01*p[2].n[0]) / dTotal;
  incenter->n[1] = (d12*p[0].n[1] + d20*p[1].n[1] + d01*p[2].n[1]) / dTotal;
  incenter->n[2] = 0;
}


// set a triangle
void SDSlowWalk::setGroundTriangle(bduVec3f* tri, bduVec3f& p1,
				   bduVec3f& p2, bduVec3f& p3)
{
  tri[0].n[0] = p1.n[0]; tri[0].n[1] = p1.n[1]; tri[0].n[2] = 0;
  tri[1].n[0] = p2.n[0]; tri[1].n[1] = p2.n[1]; tri[1].n[2] = 0;
  tri[2].n[0] = p3.n[0]; tri[2].n[1] = p3.n[1]; tri[2].n[2] = 0;
}


// inset a (2D) triangle by epsilon
void SDSlowWalk::insetTriangle(bduVec3f* p, bduVec3f* pOut, double eps)
{
  double d01, d12, d20;
  double theta0, theta1, theta2;

  d01 = sqrt(pow(p[0].n[0] - p[1].n[0], 2) + pow(p[0].n[1] - p[1].n[1], 2));
  d12 = sqrt(pow(p[1].n[0] - p[2].n[0], 2) + pow(p[1].n[1] - p[2].n[1], 2));
  d20 = sqrt(pow(p[2].n[0] - p[0].n[0], 2) + pow(p[2].n[1] - p[0].n[1], 2));

  theta0 = acos((d01*d01 + d20*d20 - d12*d12)/(2*d01*d20));
  theta1 = acos((d12*d12 + d01*d01 - d20*d20)/(2*d12*d01));
  theta2 = acos((d20*d20 + d12*d12 - d01*d01)/(2*d20*d12));

  pOut[0].n[0] = p[0].n[0] + eps/sin(theta0/2) *
    (cos(theta0/2) * (p[2].n[0] - p[0].n[0])/d20 -
     sin(theta0/2) * (p[2].n[1] - p[0].n[1])/d20);
  pOut[0].n[1] = p[0].n[1] + eps/sin(theta0/2) *
    (sin(theta0/2) * (p[2].n[0] - p[0].n[0])/d20 +
     cos(theta0/2) * (p[2].n[1] - p[0].n[1])/d20);
  pOut[0].n[2] = 0.0;

  pOut[1].n[0] = p[1].n[0] + eps/sin(theta1/2) *
    (cos(theta1/2) * (p[0].n[0] - p[1].n[0])/d01 -
     sin(theta1/2) * (p[0].n[1] - p[1].n[1])/d01);
  pOut[1].n[1] = p[1].n[1] + eps/sin(theta1/2) *
    (sin(theta1/2) * (p[0].n[0] - p[1].n[0])/d01 +
     cos(theta1/2) * (p[0].n[1] - p[1].n[1])/d01);
  pOut[1].n[2] = 0.0;

  pOut[2].n[0] = p[2].n[0] + eps/sin(theta2/2) *
    (cos(theta2/2) * (p[1].n[0] - p[2].n[0])/d12 -
     sin(theta2/2) * (p[1].n[1] - p[2].n[1])/d12);
  pOut[2].n[1] = p[2].n[1] + eps/sin(theta2/2) *
    (sin(theta2/2) * (p[1].n[0] - p[2].n[0])/d12 +
     cos(theta2/2) * (p[1].n[1] - p[2].n[1])/d12);
  pOut[2].n[2] = 0.0;
}

// find the intersection of two (2D) lines
void SDSlowWalk::intersectLines(const bduVec3f &l1p1, const bduVec3f &l1p2,
				const bduVec3f &l2p1, const bduVec3f &l2p2,
				bduVec3f *intersection)

{
  // helper variables
  double x1, x2, x3, x4, y1, y2, y3, y4;
  double t;

  x1 = l1p1.n[0]; x2 = l1p2.n[0]; x3 = l2p1.n[0]; x4 = l2p2.n[0];
  y1 = l1p1.n[1]; y2 = l1p2.n[1]; y3 = l2p1.n[1]; y4 = l2p2.n[1];

  t = ((x4-x2) - (y4-y2)*(x1-x2)/(y1-y2))/((x4-x3) - (y4-y3)*(x1-x2)/(y1-y2));
  intersection->n[0] = t*x3 + (1-t)*x4;
  intersection->n[1] = t*y3 + (1-t)*y4;
  intersection->n[2] = 0.0;
}
