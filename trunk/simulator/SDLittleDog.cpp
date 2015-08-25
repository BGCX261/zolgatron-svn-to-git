#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cassert>

#include "SDLittleDog.h"
#include "SDController.h"
#include "SDUtils.h"
#include "SDKinematics.h"
#include "../SDSlowWalk.h"

using namespace std;


// Global pointer to little dog for running drawstuff UI
SDLittleDog *gSimUIDog;
bool gSimRunning;
bool gSimPathDone = false;
int gDrawSpeed = 0;
char *cameraFile = "camera.save";

void modulateSpeed();
void saveCameraParams();
void loadCameraParams();

// Routine for displaying the contents of an SDDogState
void SDDogState::display()
{
	cout << "Dog State:" << endl;
	cout << "pos = [ " 
		 << pos.n[0] << ", " 
		 << pos.n[1] << ", "
		 << pos.n[2] << "]" << endl;
	cout << "ori = [ "
		 << ori.n[0] << ", "
		 << ori.n[1] << ", "
		 << ori.n[2] << "]" << endl;
	cout << "angRates = [ "
		 << angRates.n[0] << ", "
		 << angRates.n[1] << ", "
		 << angRates.n[2] << "]" << endl;
	for ( int i = 0; i < NUM_LEGS; i++ )
	{
		cout << "footPos[" << i << "] = [" 
			 << footPos[i].n[0] << ", "
			 << footPos[i].n[1] << ", "
			 << footPos[i].n[2] << "]" << endl;
	}
	for ( int i = 0; i < NUM_LEGS; i++ )
	{
		cout << "jointAngles[" << i << "] = [" 
			 << jointAngles[i].n[0] << ", "
			 << jointAngles[i].n[1] << ", "
			 << jointAngles[i].n[2] << "]" << endl;
	}
	cout << "time: " << timestamp << endl;		 
}

// constructor
SDLittleDog::SDLittleDog()
{
  mSimulated = true;
  mSimUI = true;
  mInitControl = false;
  mController = NULL;
  for (int i = 0; i < NUM_LEGS; i++) {
    mLegModes[i] = SD_LEG_MODE_PD;
    for (int j = 0; j < NUM_JOINTS; j++) {
      mPDGains[i*3+j][0] = 10.0;
      mPDGains[i*3+j][1] = 0.2;
    }
  }
  mIMUBias = bduVec3f(0.0, 0.0, 0.0);
}

// destructor
SDLittleDog::~SDLittleDog()
{

}

// return the current dog's state
void SDLittleDog::getDogState(SDDogState *dogState)
{
  memcpy(dogState, &mDogState, sizeof(SDDogState));
}

// update the current state of the dog
void SDLittleDog::updateDogState()
{
  bduVec4f qOri;
  bduVec3f eOri;
  bduVec3f acc;
  
  if (mSimulated) {
    // get position, orientation, angular rates, and timestamp
    mSimulator.getDogAngles(mDogState.jointAngles);
    mSimulator.getBodyInformation(LittleDog::B_TRUNK, &mDogState.pos, &qOri);
    SDQuaternionToEuler(qOri, &mDogState.ori);
    mSimulator.getDogAngularRates(&mDogState.angRates);
    mDogState.timestamp = mSimulator.getTimestamp();
  }

  // Either way, use FK now to compute foot positions
  for ( int leg = 0; leg < NUM_LEGS; leg++ )
  {
    bduVec3f localFootPos;
    SDFwdKin(leg, mDogState.jointAngles[leg], &localFootPos);
    SDLocalToWorld(mDogState.pos, mDogState.ori,
		   localFootPos, &mDogState.footPos[leg]);
  }
}


// initialize the simulator
void SDLittleDog::initializeSimulator()
{
  bduVec3f dogPos;
  bduVec4f dogOri;
  bduVec3f dogAngles[NUM_LEGS];
  bduVec3f dogFeet[NUM_LEGS];
  bduVec3f junk;
  int terId;
  bduVec3f terPos;
  bduVec4f terOri;
  int poseType;
  double servoGainMean, servoGainSig, frictionMean, frictionSig;
  int randomSeed;

  // initialize the little dog position from the simulation.params file
  fstream fin("params/simulation.params");
  if (!fin.good()) return;
  
  // read the dog position, orientation, and joint angles or feet position
  fin >> dogPos.n[0] >> dogPos.n[1] >> dogPos.n[2];
  fin >> dogOri.n[0] >> dogOri.n[1] >> dogOri.n[2] >> dogOri.n[3];
  fin >> poseType;
  if (poseType == 1) {
    for (int i = 0; i < NUM_LEGS; i++) {
      for (int j = 0; j < 3; j++) {
	fin >> dogFeet[i].n[j];
      }
      if (i == LittleDog::HL || i == LittleDog::HR) {
	SDInvKin(i, dogFeet[i], &dogAngles[i], &junk);
      } else {
	SDInvKin(i, dogFeet[i], &junk, &dogAngles[i]);
      }
    }
  } else {
    for (int i = 0; i < NUM_LEGS; i++) {
      for (int j = 0; j < NUM_JOINTS; j++) {
	fin >> dogAngles[i].n[j];
      }
    }
  }
    
  mSimulator.setDogPose(dogPos, dogOri, dogAngles);
    
  // set up the terrain boards
  fin >> terId;
  fin >> terPos.n[0] >> terPos.n[1] >> terPos.n[2];
  fin >> terOri.n[0] >> terOri.n[1] >> terOri.n[2] >> terOri.n[3];
  if (terId) {
    mSimulator.setTerrainPose(terPos, terOri);
  }

  // set the servo gain and friction
  fin >> servoGainMean >> servoGainSig >> frictionMean >> frictionSig;
  mSimulator.setServoGain(servoGainMean, servoGainSig);
  mSimulator.setFriction(frictionMean, frictionSig);

  fin >> randomSeed;
  SDSeedRandom(randomSeed);

  // simulate 100 steps to get things going
  mSimulator.setApplyControl(false);
  for (int i = 0; i < 100; i++) {
    mSimulator.step();
  }
  mSimulator.setApplyControl(true);
  
}  


  // start running the trial
LD_ERROR_CODE SDLittleDog::runTrial(SDSimulatorState *state)
{
  mInitialState = state;
  
  if (mSimulated) {
    if (mSimUI) {
      // we can only run one UI at a time
      dsFunctions fn;
      int argc = 2;
      char* argv[2] = {"littledog", "-nopause"};

      fn.version = DS_VERSION;
      fn.start = &dsSimStart;
      fn.step = &dsSimLoop;
      fn.stop = &dsSimStop;
      fn.command = &dsSimCommand;
      fn.path_to_textures = "./textures";
      
      gSimUIDog = this;
      gSimRunning = true;

      dsSimulationLoop(argc, argv, 640, 480, &fn);

    } else {
      // we should add better support for stopping
      if (simStart()) {
        gSimRunning = true;
        while (gSimRunning) {
          simStep();
        }
        simStop();
      }
    }
  }
  return LD_OKAY;
}

// Run a trial without screen flicker
LD_ERROR_CODE SDLittleDog::runPath(SDSimulatorState *state,
		vector<SDDogStep> *steps, SDSlowWalk *walkController)
{
  mInitialState = state;
	mPathSteps = steps;
	mWalkController = walkController;
  
  if ( mSimulated ) {
    if ( mSimUI ) {
      // we can only run one UI at a time
      dsFunctions fn;
      int argc = 2;
      char* argv[2] = {"littledog", "-nopause"};

      fn.version = DS_VERSION;
      fn.start = &dsSimPathStart;
      fn.step = &dsSimPathLoop;
      fn.stop = &dsSimStop;
      fn.command = &dsSimCommand;
      fn.path_to_textures = "./textures";
      
      gSimUIDog = this;
      gSimRunning = true;

      dsSimulationLoop(argc, argv, 640, 480, &fn);

    } else {
			printf("Should not be asking runPath to run not in UI mode.\n");
			assert(false);
    }
  }
  return LD_OKAY;
}


// stop the trial
LD_ERROR_CODE SDLittleDog::stopRobot(void)
{
  gSimRunning = false;
  return LD_OKAY;
}


// perform all functions necessary for starting a trial in simulation
bool SDLittleDog::simStart()
{
  return initControl();
}

// take a step in simulation
void SDLittleDog::simStep()
{
  updateControl();
  mSimulator.step();
}

// stop the simulation
void SDLittleDog::simStop()
{
  uninitControl();
}

// draw the simulation
void SDLittleDog::simDraw()
{
  mSimulator.dsDraw();
  mController->dsDraw();
}

// *** Functions for drawing a path instead of one step ***

// perform all functions necessary for starting a trial in simulation
bool SDLittleDog::simPathStart()
{
	// Prime walk controller with first step.
	SDDogStep step = mPathSteps->back();
	mWalkController->setStepPosition(step.foot, step.x, step.y);
	step.print();
	mPathSteps->pop_back();

  return initControl();
}

// For a multi-path simulation, check for another step after we have finished
// one. Return true if still running.
bool SDLittleDog::simPathCycle()
{
	if ( (int)mPathSteps->size() > 0 ) {
		// Do computation steps between path steps.
		uninitControl(); // This is a do-nothing, however.
		
		// This prevents the simulator straying from search.
		SDSimulatorState state;
    mSimulator.getState(&state);
    mSimulator.setState(&state);
		// And get state again, as that is how it is done.
    mSimulator.getState(&state);
		// Set new initial state.
		mInitialState = &state;

		// Re-start trial.
		simPathStart();
		return true;
	} else {
		return false;
	}
}


// determine whether a given leg hit the ground
bool SDLittleDog::legHit(int leg)
{
  return mSimulator.legHit(leg);
}


// set up the trial
bool SDLittleDog::initControl()
{
    
  mSimulator.setState(mInitialState);
  
  mController->setLittleDog(this);
  mController->reset();
  
  mInitControl = true;
  return true;
}

// update the control for one step
void SDLittleDog::updateControl()
{
  bduVec3f dogAngles[NUM_LEGS];

  // initialize the controller if necessary, or advance the joints
  if (mInitControl) {
    mInitControl = false;
  } else {
    memcpy(&mOldFootForces, &mFootForces, sizeof(bduVec4f));
    mController->advanceAngles();
  }

  // set the desired angles
  updateDogState();
  mController->getAngles(dogAngles);
  if (mSimulated) {
    mSimulator.setDogDesiredAngles(dogAngles);
  }
}

// uninitialize the control
void SDLittleDog::uninitControl()
{
  
}

void loadCameraParams()
{
	FILE *f = fopen(cameraFile, "r");
  static float xyz[3];
  static float hpr[3];
  if (f!=NULL) {
    fread(&xyz, sizeof(float), 3, f);
    fread(&hpr, sizeof(float), 3, f);
    dsSetViewpoint (xyz,hpr);
    fclose(f);
  }
} 

// drawstuff callback at start of simulation - load saved camera
void dsSimStart()
{
  loadCameraParams();
  gSimUIDog->simStart();
}

void saveCameraParams()
{
	FILE *f = fopen(cameraFile, "w");
  float xyz[3], hpr[3];
  dsGetViewpoint(xyz, hpr);
  fwrite(&xyz, sizeof(float), 3, f);
  fwrite(&hpr, sizeof(float), 3, f);
  fclose(f);
}

// drawstuff callback to end simulation
void dsSimStop()
{
	saveCameraParams();
  gSimUIDog->simStop();
}

// drawstuff simulation loop callback
void dsSimLoop(int pause)
{
  if (gSimRunning) {
    if (!pause) {
      gSimUIDog->simStep();
    }
  } else {
    dsStop();
  }
  gSimUIDog->simDraw();
}

// Function to alter the playback speed.
void modulateSpeed()
{
	int speedStep = 20, maxSpeed = 60;
	if ( gDrawSpeed == 0 ) gDrawSpeed = 1;
	else if ( gDrawSpeed == 1 ) gDrawSpeed = speedStep;
	else if ( gDrawSpeed < maxSpeed ) gDrawSpeed += speedStep;
	else if ( gDrawSpeed == maxSpeed ) gDrawSpeed = 0;
	// Print message about display speed.
	if ( gDrawSpeed == 1 ) printf("Drawing every frame.\n");
	else if ( gDrawSpeed > 0 ) printf("Drawing every %dth frame.\n", gDrawSpeed);
	else printf("Drawing only inter-step frames.\n");
}

// drawstuff command callback
void dsSimCommand(int cmd)
{
  bool dynamics;
  switch (cmd) {
		case 's':
			modulateSpeed();
			break;
		case 'f':
			gDrawSpeed = 0;
			printf("Drawing only inter-step frames.\n");
			break;
		case 'v':
			saveCameraParams();
			printf("Saved camera parameters to '%s'.\n", cameraFile);
			break;
		case 'd':
		  dynamics = gSimUIDog->getSimulator()->getDynamicsEnabled();
		  gSimUIDog->getSimulator()->setDynamicsEnabled(!dynamics);
		  break;
		case 't':
		  SDSetDrawTerrain(!SDGetDrawTerrain());
		  break;
  }
}

// *** Functions for running a path of steps without flicker ***

// drawstuff callback at start of simulation - load saved camera
void dsSimPathStart()
{
	loadCameraParams();
  gSimUIDog->simPathStart();
}


// drawstuff simulation loop callback
void dsSimPathLoop(int pause)
{
  if (gSimRunning) {
    if (!pause) {
			if ( gDrawSpeed == 0 ) {
				// Keep simulating until the dog has finished its step and gSimRunning goes false.
				while (gSimRunning) gSimUIDog->simStep();
			} else {
				// Simulator gDrawSpeed steps before rendering a frame.
				for ( int i=gDrawSpeed; i>0 && gSimRunning; --i ) {
					gSimUIDog->simStep();
				}
			}
    }
  } else {
		// We have finished step.
		gSimRunning = gSimUIDog->simPathCycle();
		if ( !gSimRunning && !gSimPathDone ) {
			gSimPathDone = true;
			printf("Done following path.\n");
	    //dsStop(); // Exit simulation
		}
  }
  gSimUIDog->simDraw();
}

// Parse a path file and return a vector of steps.
bool parsePath(string file, vector<SDDogStep> *steps) {
	// Parse all path steps.
	vector<SDDogStep> tempSteps;
  string line;
  ifstream paths;
  paths.open(file.c_str());
  if ( !paths.is_open() ) {
    printf(("Error opening "+file+"\n").c_str());
    return false;
  }
  while ( !paths.eof() ) {
    getline(paths,line);
		if ( line.size() == 0 ) break;
    int index = line.find(" ");
    int foot = atoi((line.substr(0,index)).c_str());
    int index2 = line.find(" ",index+1);
    double x = atof((line.substr(index+1,index2-index-1)).c_str());
    double y = atof((line.substr(index2)).c_str());
		tempSteps.push_back(SDDogStep(foot, x, y));
  }
  paths.close();
	// Reverse step order
	steps->clear();
	steps->reserve(tempSteps.size());
	while ( tempSteps.size() > 0 ) {
		steps->push_back(tempSteps.back());
		tempSteps.pop_back();
	}
	return true;
}



