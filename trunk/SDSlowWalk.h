#ifndef __SD_SLOW_WALK_H__
#define __SD_SLOW_WALK_H__

#include "SDController.h"

enum {
  SD_SLOW_WALK_SHIFT_CG,
  SD_SLOW_WALK_STEP,
  SD_SLOW_WALK_REPOSITION,
};

class SDSlowWalk : public SDController
{
 public:
  SDSlowWalk() {};
  ~SDSlowWalk() {};

  virtual void getAngles(bduVec3f *angles);
  virtual void advanceAngles() {};
  virtual bool reset();
  virtual void dsDraw();

  void setStepPosition(int movingFoot, double x, double y);

  void getFeetCenter(bduVec3f *feet, bduVec3f *center);

 private:
  void beginShiftCG();
  void beginStep();
  void beginReposition();
  
  void getNextFootstep(int *movingFoot, bduVec3f *globalStepPos);
  void getSupportTriangle(bduVec3f *feet, bduVec3f *tri);
  void getMarginCenter(const bduVec3f &feetCenter,
		       const bduVec3f *supportTri, bduVec3f *p);
  void getTriangleIncenter(const bduVec3f* p, bduVec3f *incenter);
  void setGroundTriangle(bduVec3f* tri, bduVec3f& p1,
			 bduVec3f& p2, bduVec3f& p3);
  void insetTriangle(bduVec3f* p, bduVec3f* pOut, double eps);
  void intersectLines(const bduVec3f &l1p1, const bduVec3f &l1p2,
		      const bduVec3f &l2p1, const bduVec3f &l2p2,
		      bduVec3f *intersection);

  bduVec3f mFootPos[NUM_LEGS], mNextFootPos[NUM_LEGS];
  bduVec3f mGlobalFootPos[NUM_LEGS];
  bduVec3f mGlobalStepPos;
  bduVec3f mStepWaypoints[3];
  int mControllerMode;
  int mMovingFoot;
  double mShiftCGTime;
  double mStepTime;
  double mWaypointTime[3];
  double mStartTime;
  SDDogState mCurrentState;

  bduVec3f gTri[3];
  bduVec3f gCenter;

  

  // parameters
  double mBodySpeed;
  double mFootSpeed;
  double mFootHeight;
  double mStepHeight;
  double mCGMargin;
  double mPauseTime;
  double mRepositionTime;
};


#endif


