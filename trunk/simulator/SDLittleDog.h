#ifndef __SD_LITTLE_DOG_H__
#define __SD_LITTLE_DOG_H__

#include <littledog.h>
#include "SDSimulator.h"
#include "SDConstants.h"
#include <vector>

// Forward declaration of slow walk controller.
class SDSlowWalk;

// structure to encapsulate all state information
struct SDDogState {
	bduVec3f pos;
	bduVec3f ori;
	bduVec3f angRates;
	bduVec3f footPos[NUM_LEGS];
	bduVec3f jointAngles[NUM_LEGS];
	double timestamp;

	void display();
};


enum SDLegMode {
  SD_LEG_MODE_PD = 0,
  SD_LEG_MODE_COMPLIANT,
};


// Struct for holding dog steps
struct SDDogStep {
	int foot;
	double x, y;
	SDDogStep(int foot, double x, double y) : foot(foot), x(x), y(y) {}
	void print() {
		printf("Step %d: %0.3f, %0.3f\n", foot, x, y);
	}
};


class SDController;


/*
  The SDLittleDog class encapsulates all the logic for setting up and
  running the Little Dog, either in simulation or on the real robot.
*/
class SDLittleDog
{
 public:
  SDLittleDog();
  ~SDLittleDog();

  void getDogState(SDDogState *dogState);
  void updateDogState();

  LD_ERROR_CODE runTrial(SDSimulatorState *state);
  LD_ERROR_CODE stopRobot(void);
  LD_ERROR_CODE runPath(SDSimulatorState *state, vector<SDDogStep> *steps,
			SDSlowWalk *walkController);

  bool simStart();
  void simStep();
  void simStop();
  void simDraw();

  bool simPathStart();
	bool simPathCycle();

  void initializeSimulator();

  bool legHit(int leg);

  // accessors and observers
  bool getSimulated()
    { return mSimulated; }
  void setSimulated(bool simulated)
    { mSimulated = simulated; }

  bool getSimUI()
    { return mSimUI; }
  void setSimUI(bool simUI)
    { mSimUI = simUI; }
  
  SDController* getController()
    { return mController; }
  void setController(SDController *controller)
    { mController = controller; }

  SDSimulator* getSimulator()
    { return &mSimulator; }

  void setLegMode(int leg, SDLegMode mode)
    { mLegModes[leg] = mode; }
  SDLegMode getLegMode(int leg)
    { return mLegModes[leg]; }

 protected:
  bool initControl(void);
  void updateControl(void);
  void uninitControl(void);


 private:
  SDDogState mDogState;
  SDSimulator mSimulator;
  SDController *mController;
  bduVec4f mFootForces, mOldFootForces;
  SDLegMode mLegModes[NUM_LEGS];
  double mPDGains[NUM_LEGS*NUM_JOINTS][2];
  bduVec3f mIMUBias;

  SDSimulatorState *mInitialState;
	vector<SDDogStep> *mPathSteps;
	SDSlowWalk *mWalkController;
  
  bool mSimulated;
  bool mSimUI;
  bool mInitControl;
};



void dsSimStart();
void dsSimStop();
void dsSimCommand(int cmd);
void dsSimLoop(int pause);

void dsSimPathStart();
void dsSimPathLoop(int pause);

bool parsePath(string file, vector<SDDogStep> *steps);

#endif



