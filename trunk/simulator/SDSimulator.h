#ifndef __SD_SIMULATOR_H__
#define __SD_SIMULATOR_H__

#include <ode/ode.h>
#include <pthread.h>
#include "bduVec3f.h"
#include "bduVec4f.h"
#include "SDTerrain.h"
#include "SDConstants.h"
#include <fstream>

using namespace std;

enum {
  SD_COLLIDE_FL = 0x0001,
  SD_COLLIDE_FR = 0x0002,
  SD_COLLIDE_BL = 0x0004,
  SD_COLLIDE_BR = 0x0008
};

struct SDSimulatorState {
  double timestamp;
  double bodyStates[13][13];
  double jointVelocities[12];
};

class SDSimulator
{
 public:
  SDSimulator();
  ~SDSimulator();

  void step();
  void collide(dGeomID o1, dGeomID o2);

  bool getDynamicsEnabled();
  void setDynamicsEnabled(bool dynamicsEnabled);

  void setDogPose(bduVec3f &pos, bduVec4f &ori, bduVec3f* angles);
  void setDogPosition(bduVec3f &pos);
  void setDogOrientation(bduVec4f &ori);
  void setDogAngles(bduVec3f *angles);
  void setDogDesiredAngles(bduVec3f *angles);
  void clearDogVelocity();
  void getDogAngles(bduVec3f *angles);
  void getDogAngularRates(bduVec3f *angRates);
  bool legHit(int leg);

  void setTerrainPose(bduVec3f &pos, bduVec4f &ori);
  void setTerrain();
  void setTerrainPosition(bduVec3f &pos);
  void setTerrainOrientation(bduVec4f &orient);
  void saveTerrainHeightmapFile();
  
  SDTerrain* getTerrain();
  double getPointHeight(double x, double y);
  void getBodyInformation(int body, bduVec3f *pos, bduVec4f *ori) const;
  void dsDraw();

  void setServoGain(double servoGainMean, double servoGainSig);
  void setFriction(double frictionMean, double frictionSig);

  void setPlaybackFile(char* filename);
  void writeStateToFile(char* filename);
  void loadStateFromFile(char* filename);
  void getState(SDSimulatorState *state);
  void setState(const SDSimulatorState *state);

  void getLocalFootPositions(bduVec3f *localFeet);
  void getGlobalFootPositions(bduVec3f *globalFeet);
  void getLocalStepPosition(double x, double y, bduVec3f *localStep);
  

  // accessors and observers
  bool getApplyControl()
    { return mApplyControl; }
  void setApplyControl(bool applyControl)
    { mApplyControl = applyControl; }

  double getTimestamp()
    { return mTimestamp; }

  void setCollisionMask(int collisionMask)
    { mCollisionMask = collisionMask; }

 private:
  void loadCurrentPlaybackFrame();
  void readBodyInfo(dBodyID body, ifstream &fin);
  void writeBodyInfo(dBodyID body, ofstream &fout);
  void getBodyState(dBodyID body, double *state);
  void setBodyState(dBodyID body, const double *state);
  
  void buildLittleDogModel(void);


  struct ODEEnvironment
  {
    dWorldID world;
    dSpaceID littleDogSpace;
    dSpaceID terrainSpace;
    dJointGroupID contactGroup;
    dGeomID ground;

    double servoGainMean, servoGainSig;
    double frictionMean, frictionSig;
  };

  struct LittleDogODEModel
  {
    dBodyID trunk;
    dBodyID hip[4];
    dBodyID uleg[4];
    dBodyID lleg[4];
    
    dGeomID trunkGeom;
    dGeomID hipGeom[4];
    dGeomID ulegGeom[4];
    dGeomID llegGeom[4];
    
    dJointID hipRx[4];
    dJointID hipRy[4];
    dJointID kneeRy[4];
  };


  ODEEnvironment mOde;
  LittleDogODEModel mLittleDog;
  SDTerrain *mTerrain;
  bduVec3f mDesiredAngles[4];
  bool mLegContacts[4];

  int mNumCollisions;
  bool mApplyControl;
  bool mDynamicsEnabled;
  double mTimestamp;
  int mCollisionMask;
  bool mPlayback;
  int mPlaybackIndex;
  char mPlaybackFile[256];

  pthread_mutex_t mSimMutex;

  const SDConstants& mSdc;
};


void odeNearCallback(void *data, dGeomID o1, dGeomID o2);

#endif
