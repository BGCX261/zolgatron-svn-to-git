#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "SDSimulator.h"
#include "SDLittleDog.h"
#include "SDConstants.h"
#include "SDUtils.h"
#include "SDKinematics.h"

using namespace std;

// constructor
SDSimulator::SDSimulator() :
	mSdc(SDConstants::getInstance())
{

  // Set up ODE
  mOde.world = dWorldCreate();
  mOde.littleDogSpace = dHashSpaceCreate(0);
  mOde.terrainSpace = dHashSpaceCreate(0);
  mOde.contactGroup = dJointGroupCreate(0);
  mOde.ground = dCreatePlane(mOde.terrainSpace, 0, 0, 1, 0.0);
  
  mOde.servoGainMean = mSdc.mODEServoGain;
  mOde.servoGainSig = 0.0;
  mOde.frictionMean = mSdc.mODEFriction;
  mOde.frictionSig = 0.0;
  
  dWorldSetGravity(mOde.world, 0, 0, -9.8);
  dWorldSetCFM(mOde.world, mSdc.mODEWorldCFM);
  dWorldSetERP(mOde.world, mSdc.mODEWorldERP);

  pthread_mutex_init(&mSimMutex, NULL);

  // clear the terrain
  mTerrain = 0;

  // zero the initial angles
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      mDesiredAngles[i].n[j] = 0.0;
    }
  }
  
  mApplyControl = true;
  mDynamicsEnabled = true;
  mPlayback = false;
  mTimestamp = 0.0;
  mCollisionMask = SD_COLLIDE_FL | SD_COLLIDE_FR | SD_COLLIDE_BL |
    SD_COLLIDE_BR;

  // Build the little dog model
  buildLittleDogModel();

}

// destructor
SDSimulator::~SDSimulator()
{
  dWorldDestroy(mOde.world);
  dSpaceDestroy(mOde.littleDogSpace);
  dSpaceDestroy(mOde.terrainSpace);
  dJointGroupDestroy(mOde.contactGroup);
}

// apply PD control and take a step in simulation
void SDSimulator::step()
{
  double hipRxVel, hipRyVel, kneeRyVel;
  double hipRxDelta, hipRyDelta, kneeRyDelta;

  pthread_mutex_lock(&mSimMutex);
  
  // apply the control
  if (mApplyControl) {
    for (int i = 0; i < NUM_LEGS; i++) {
      hipRxDelta = (mDesiredAngles[i].n[0] + mSdc.mJointAngleOffsets[i*3+0] -
		    dJointGetHingeAngle(mLittleDog.hipRx[i]));
      hipRyDelta = (mDesiredAngles[i].n[1] + mSdc.mJointAngleOffsets[i*3+1] -
		    dJointGetHingeAngle(mLittleDog.hipRy[i]));
      kneeRyDelta = (mDesiredAngles[i].n[2] + mSdc.mJointAngleOffsets[i*3+2] -
		     dJointGetHingeAngle(mLittleDog.kneeRy[i]));

      hipRxVel = SDRandomNormal(mOde.servoGainMean, mOde.servoGainSig)
	* hipRxDelta;
      hipRyVel = SDRandomNormal(mOde.servoGainMean, mOde.servoGainSig)
	* hipRyDelta;
      kneeRyVel = SDRandomNormal(mOde.servoGainMean, mOde.servoGainSig)
	* kneeRyDelta;

      dJointSetHingeParam(mLittleDog.hipRx[i], dParamVel, hipRxVel);
      dJointSetHingeParam(mLittleDog.hipRy[i], dParamVel, hipRyVel);
      dJointSetHingeParam(mLittleDog.kneeRy[i], dParamVel, kneeRyVel);
    }
  }

  // handle collisions and take a simulation step
  if (mDynamicsEnabled) {
    mNumCollisions = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
      mLegContacts[i] = false;
    }
    
    dSpaceCollide2((dGeomID)mOde.littleDogSpace, (dGeomID)mOde.terrainSpace,
		   (void*)this, &odeNearCallback);
  }

  dWorldStep(mOde.world, mSdc.mODEWorldStep);
  dJointGroupEmpty(mOde.contactGroup);
  mTimestamp += mSdc.mODEWorldStep;

  if (mPlayback) {
    mPlaybackIndex++;
    loadCurrentPlaybackFrame();
  }
  pthread_mutex_unlock(&mSimMutex);
}

// Callback for ODE collision detection
void odeNearCallback(void *data, dGeomID o1, dGeomID o2)
{
  SDSimulator *simulator = (SDSimulator*)data;
  simulator->collide(o1, o2);
}

// handle collision between two geoms
void SDSimulator::collide(dGeomID o1, dGeomID o2)
{
  dContact contact[mSdc.mODEContactPoints];
  int n;

  if (((o1 == mLittleDog.llegGeom[0] || o2 == mLittleDog.llegGeom[0]) &&
       mCollisionMask & SD_COLLIDE_FL) ||
      ((o1 == mLittleDog.llegGeom[1] || o2 == mLittleDog.llegGeom[1]) &&
       mCollisionMask & SD_COLLIDE_FR) ||
      ((o1 == mLittleDog.llegGeom[2] || o2 == mLittleDog.llegGeom[2]) &&
       mCollisionMask & SD_COLLIDE_BL) ||
      ((o1 == mLittleDog.llegGeom[3] || o2 == mLittleDog.llegGeom[3]) &&
       mCollisionMask & SD_COLLIDE_BR)) {


    n = dCollide(o1, o2, mSdc.mODEContactPoints,
		 &contact[0].geom, sizeof(dContact));

    if ((o1 == mLittleDog.llegGeom[0]
	 || o2 == mLittleDog.llegGeom[0]) && n > 0)
      mLegContacts[0] = true;
    if ((o1 == mLittleDog.llegGeom[1]
	 || o2 == mLittleDog.llegGeom[1]) && n > 0)
      mLegContacts[1] = true;
    if ((o1 == mLittleDog.llegGeom[2]
	 || o2 == mLittleDog.llegGeom[2]) && n > 0)
      mLegContacts[2] = true;
    if ((o1 == mLittleDog.llegGeom[3]
	 || o2 == mLittleDog.llegGeom[3]) && n > 0)
      mLegContacts[3] = true;
    

    mNumCollisions += n;
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM |
	dContactApprox1;
      contact[i].surface.mu = SDRandomNormal(mOde.frictionMean,
					     mOde.frictionSig);
      contact[i].surface.soft_erp = mSdc.mODEContactERP;
      contact[i].surface.soft_cfm = mSdc.mODEContactCFM;
      contact[i].surface.bounce = 0.001;
      dJointID c = dJointCreateContact(mOde.world, mOde.contactGroup,
				       &contact[i]);
      dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
    }
  }
}

// return whether dynamics are enabled
bool SDSimulator::getDynamicsEnabled()
{
  return mDynamicsEnabled;
}

// set whether dynamics are enabled
void SDSimulator::setDynamicsEnabled(bool dynamicsEnabled)
{
  mDynamicsEnabled = dynamicsEnabled;
  if (dynamicsEnabled) {
    dWorldSetGravity(mOde.world, 0, 0, -9.8);
  } else {
    dWorldSetGravity(mOde.world, 0, 0, 0.0);
  }

  // zero the velocities
  dBodySetAngularVel(mLittleDog.trunk, 0, 0, 0);
  dBodySetLinearVel(mLittleDog.trunk, 0, 0, 0);
  for (int i = 0; i < NUM_LEGS; i++) {
    dBodySetAngularVel(mLittleDog.hip[i], 0, 0, 0);
    dBodySetLinearVel(mLittleDog.hip[i], 0, 0, 0);
    dBodySetAngularVel(mLittleDog.uleg[i], 0, 0, 0);
    dBodySetLinearVel(mLittleDog.uleg[i], 0, 0, 0);
    dBodySetAngularVel(mLittleDog.lleg[i], 0, 0, 0);
    dBodySetLinearVel(mLittleDog.lleg[i], 0, 0, 0);
  }

}

void SDSimulator::setDogPose(bduVec3f &pos, bduVec4f &ori, bduVec3f* angles)
{
  // set the desired position and orientation
  setDogPosition(pos);
  setDogOrientation(ori);
  setDogAngles(angles);
}

// set the position of the dog
void SDSimulator::setDogPosition(bduVec3f &pos)
{
  const dReal *trunkPos, *oldPos;
  dVector3 newPos;
  trunkPos = dBodyGetPosition(mLittleDog.trunk);

  for (int i = 0; i < NUM_LEGS; i++) {
    oldPos = dBodyGetPosition(mLittleDog.hip[i]);
    newPos[0] = oldPos[0] - trunkPos[0] + pos.n[0];
    newPos[1] = oldPos[1] - trunkPos[1] + pos.n[1];
    newPos[2] = oldPos[2] - trunkPos[2] + pos.n[2];
    dBodySetPosition(mLittleDog.hip[i], newPos[0], newPos[1], newPos[2]);

    oldPos = dBodyGetPosition(mLittleDog.uleg[i]);
    newPos[0] = oldPos[0] - trunkPos[0] + pos.n[0];
    newPos[1] = oldPos[1] - trunkPos[1] + pos.n[1];
    newPos[2] = oldPos[2] - trunkPos[2] + pos.n[2];
    dBodySetPosition(mLittleDog.uleg[i], newPos[0], newPos[1], newPos[2]);

    oldPos = dBodyGetPosition(mLittleDog.lleg[i]);
    newPos[0] = oldPos[0] - trunkPos[0] + pos.n[0];
    newPos[1] = oldPos[1] - trunkPos[1] + pos.n[1];
    newPos[2] = oldPos[2] - trunkPos[2] + pos.n[2];
    dBodySetPosition(mLittleDog.lleg[i], newPos[0], newPos[1], newPos[2]);
  }

  dBodySetPosition(mLittleDog.trunk, pos.n[0], pos.n[1], pos.n[2]);
}


// set the orientation of the little dog
void SDSimulator::setDogOrientation(bduVec4f &orient)
{
  dQuaternion desiredOri, trunkRot, bodyPosQ, newBodyPosQ, temp, newBodyOri;
  const dReal *trunkOri, *bodyOri, *bodyPos, *trunkPos;
  dBodyID body;

  trunkOri = dBodyGetQuaternion(mLittleDog.trunk);

  desiredOri[0] = orient.n[3];
  desiredOri[1] = orient.n[0];
  desiredOri[2] = orient.n[1];
  desiredOri[3] = orient.n[2];
  
  // get the position, orientation, and desired rotation of the trunk
  trunkPos = dBodyGetPosition(mLittleDog.trunk);
  dQMultiply2(trunkRot, desiredOri, trunkOri);

  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      switch (j) {
      case 0: body = mLittleDog.hip[i]; break;
      case 1: body = mLittleDog.uleg[i]; break;
      case 2: body = mLittleDog.lleg[i]; break;
      }
	
      // rotate and reposition the body
      bodyOri = dBodyGetQuaternion(body);
      bodyPos = dBodyGetPosition(body);

      bodyPosQ[0] = 0.0;
      bodyPosQ[1] = bodyPos[0] - trunkPos[0];
      bodyPosQ[2] = bodyPos[1] - trunkPos[1];
      bodyPosQ[3] = bodyPos[2] - trunkPos[2];

      dQMultiply0(temp, trunkRot, bodyPosQ);
      dQMultiply2(newBodyPosQ, temp, trunkRot);

      dQMultiply0(newBodyOri, trunkRot, bodyOri);
      
      dBodySetPosition(body, newBodyPosQ[1] + trunkPos[0],
      	       newBodyPosQ[2] + trunkPos[1], newBodyPosQ[3] + trunkPos[2]);
      dBodySetQuaternion(body, newBodyOri);
    }
  }
  
  // rotate the trunk
  dBodySetQuaternion(mLittleDog.trunk, desiredOri);
}

// set the joint angles directly
void SDSimulator::setDogAngles(bduVec3f *angles)
{
  const dReal *trunkOri, *trunkPos;
  dQuaternion qHipX, qHipY, qKneeY, qHip, qULeg, qLLeg;
  dQuaternion pHipX, pHipY, pKneeY, pFoot1, pFoot2, temp;
  dVector3 pHip, pULeg, pLLeg;
  
  trunkPos = dBodyGetPosition(mLittleDog.trunk);
  trunkOri = dBodyGetQuaternion(mLittleDog.trunk);

  for (int i = 0; i < NUM_LEGS; i++) {
    double xSign = (i < 2 ? 1.0 : -1.0);
    double ySign = (i % 2 == 0 ? 1.0 : -1.0);
    
    qHipX[0] = cos(angles[i].n[0]/2);
    qHipX[1] = sin(angles[i].n[0]/2);
    qHipX[2] = 0;
    qHipX[3] = 0;
    
    qHipY[0] = cos(angles[i].n[1]/2);
    qHipY[1] = 0;
    qHipY[2] = sin(angles[i].n[1]/2);
    qHipY[3] = 0;
    
    qKneeY[0] = cos(angles[i].n[2]/2);
    qKneeY[1] = 0;
    qKneeY[2] = sin(angles[i].n[2]/2);
    qKneeY[3] = 0;
    
    pHipX[0] = 0;
    pHipX[1] = xSign * mSdc.mLDHipRxOffsetX;
    pHipX[2] = ySign * mSdc.mLDHipRxOffsetY;
    pHipX[3] = mSdc.mLDHipRxOffsetZ;
    
    pHipY[0] = 0;
    pHipY[1] = xSign * mSdc.mLDHipRyOffsetX;
    pHipY[2] = ySign * mSdc.mLDHipRyOffsetY;
    pHipY[3] = mSdc.mLDHipRyOffsetZ;
    
    pKneeY[0] = 0;
    pKneeY[1] = xSign * mSdc.mLDKneeRyOffsetX;
    pKneeY[2] = ySign * mSdc.mLDKneeRyOffsetY;
    pKneeY[3] = mSdc.mLDKneeRyOffsetZ;
    
    pFoot1[0] = 0;
    pFoot1[1] = xSign * mSdc.mLDFootOffsetX;
    pFoot1[2] = ySign * mSdc.mLDFootOffsetY;
    pFoot1[3] = 0;
    
    pFoot2[0] = 0;
    pFoot2[1] = 0;
    pFoot2[2] = 0;
    pFoot2[3] = mSdc.mLDFootOffsetZ;
    
    dQMultiply0(qHip, trunkOri, qHipX);
    dQMultiply0(qULeg, qHip, qHipY);
    dQMultiply0(qLLeg, qULeg, qKneeY);
    
    dQMultiply0(temp, trunkOri, pHipX); dQMultiply2(pHipX, temp, trunkOri);
    dQMultiply0(temp, qHip, pHipY); dQMultiply2(pHipY, temp, qHip);
    dQMultiply0(temp, qULeg, pKneeY); dQMultiply2(pKneeY, temp, qULeg);
    dQMultiply0(temp, qLLeg, pFoot1); dQMultiply2(pFoot1, temp, qLLeg);
    dQMultiply0(temp, qLLeg, pFoot2); dQMultiply2(pFoot2, temp, qLLeg);
    
    pHip[0] = trunkPos[0] + pHipX[1] + pHipY[1]/2.0;
    pHip[1] = trunkPos[1] + pHipX[2] + pHipY[2]/2.0;
    pHip[2] = trunkPos[2] + pHipX[3] + pHipY[3]/2.0;
    
    pULeg[0] = pHip[0] + pHipY[1]/2.0 + pKneeY[1]/2.0;
    pULeg[1] = pHip[1] + pHipY[2]/2.0 + pKneeY[2]/2.0;
    pULeg[2] = pHip[2] + pHipY[3]/2.0 + pKneeY[3]/2.0;
    
    pLLeg[0] = pULeg[0] + pKneeY[1]/2.0 + pFoot1[1] + pFoot2[1]/2.0;
    pLLeg[1] = pULeg[1] + pKneeY[2]/2.0 + pFoot1[2] + pFoot2[2]/2.0;
    pLLeg[2] = pULeg[2] + pKneeY[3]/2.0 + pFoot1[3] + pFoot2[3]/2.0;

    dBodySetPosition(mLittleDog.hip[i], pHip[0], pHip[1], pHip[2]);
    dBodySetQuaternion(mLittleDog.hip[i], qHip);
    dBodySetPosition(mLittleDog.uleg[i], pULeg[0], pULeg[1], pULeg[2]);
    dBodySetQuaternion(mLittleDog.uleg[i], qULeg);
    dBodySetPosition(mLittleDog.lleg[i], pLLeg[0], pLLeg[1], pLLeg[2]);
    dBodySetQuaternion(mLittleDog.lleg[i], qLLeg);
  }
}

  

// set the desired joint angles for the dog
void SDSimulator::setDogDesiredAngles(bduVec3f *angles)
{
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      mDesiredAngles[i].n[j] = angles[i].n[j];
    }
  }
}


// make sure the dog has no built up velocity
void SDSimulator::clearDogVelocity()
{
  dBodySetLinearVel(mLittleDog.trunk, 0, 0, 0);
  dBodySetAngularVel(mLittleDog.trunk, 0, 0, 0);

  for (int i = 0; i < NUM_LEGS; i++) {
    dBodySetLinearVel(mLittleDog.hip[i], 0, 0, 0);
    dBodySetAngularVel(mLittleDog.hip[i], 0, 0, 0);
    dBodySetLinearVel(mLittleDog.uleg[i], 0, 0, 0);
    dBodySetAngularVel(mLittleDog.uleg[i], 0, 0, 0);
    dBodySetLinearVel(mLittleDog.lleg[i], 0, 0, 0);
    dBodySetAngularVel(mLittleDog.lleg[i], 0, 0, 0);
  }
}

// return the current angles of the dog
void SDSimulator::getDogAngles(bduVec3f *angles)
{
  for (int i = 0; i < NUM_LEGS; i++) {
    angles[i].n[0] = dJointGetHingeAngle(mLittleDog.hipRx[i]);
    angles[i].n[1] = dJointGetHingeAngle(mLittleDog.hipRy[i]);
    angles[i].n[2] = dJointGetHingeAngle(mLittleDog.kneeRy[i]);
  }
}

// get the angular rate of the dog's body
void SDSimulator::getDogAngularRates(bduVec3f *angRates)
{
  const dReal* angVel;
  angVel = dBodyGetAngularVel(mLittleDog.trunk);
  angRates->n[0] = angVel[0];
  angRates->n[1] = angVel[1];
  angRates->n[2] = angVel[2];
}

// determine if a given leg is touching the ground
bool SDSimulator::legHit(int leg)
{
  return mLegContacts[leg];
}


// set all the terrain information: id, position, and orientation
void SDSimulator::setTerrainPose(bduVec3f &pos, bduVec4f &ori)
{
  setTerrain();
  setTerrainPosition(pos);
  setTerrainOrientation(ori);
}

// set the terrain for index
void SDSimulator::setTerrain()
{
  if (mTerrain != 0) {
    delete mTerrain;
  }
  mTerrain = new SDTerrain("tergen/terrain.trimesh");
  mTerrain->buildInSpace(mOde.terrainSpace);
}


// set the position of a terrain board
void SDSimulator::setTerrainPosition(bduVec3f &pos)
{
  dVector3 pos2;
  pos2[0] = pos.n[0];
  pos2[1] = pos.n[1];
  pos2[2] = pos.n[2];

  if (mTerrain != 0) {
    mTerrain->setPosition(pos2);
  }
}

// set the orientation of a terrain board
void SDSimulator::setTerrainOrientation(bduVec4f &orient)
{
  dQuaternion orient2;

  orient2[0] = orient.n[3];
  orient2[1] = orient.n[0];
  orient2[2] = orient.n[1];
  orient2[3] = orient.n[2];

  if (mTerrain != 0) {
    mTerrain->setOrientation(orient2);
  }
}


// get the terrain object
SDTerrain* SDSimulator::getTerrain()
{
  return mTerrain;
}


// get the height of a point
double SDSimulator::getPointHeight(double x, double y)
{
  dGeomID ray;
  dContactGeom contact[mSdc.mODERayContactPoints];
  int n;

  
  pthread_mutex_lock(&mSimMutex);

  ray = dCreateRay(mOde.littleDogSpace, 1.01);
  dGeomRaySet(ray, x, y, 1.0, 0, 0, -1.0);

  n = dCollide(ray, (dGeomID)mOde.terrainSpace, mSdc.mODERayContactPoints,
	       contact, sizeof(dContactGeom));
  double max_height = -1.0;
  for (int i = 0; i < n; i++) {
    if (contact[i].pos[2] > max_height) {
      max_height = contact[i].pos[2];
    }
  }
  dGeomDestroy(ray);
  pthread_mutex_unlock(&mSimMutex);
  
  return max_height;
}

// return the position and orientation of a body
void SDSimulator::getBodyInformation(int body, bduVec3f *pos,
				     bduVec4f *ori) const
{
  const dReal *odePos;
  const dReal *odeOri;
  dBodyID b;

  switch(body) {
    case LittleDog::B_TRUNK: b = mLittleDog.trunk; break;
    case LittleDog::B_FL_ULEG: b = mLittleDog.uleg[0]; break;
    case LittleDog::B_FR_ULEG: b = mLittleDog.uleg[1]; break;
    case LittleDog::B_HL_ULEG: b = mLittleDog.uleg[2]; break;
    case LittleDog::B_HR_ULEG: b = mLittleDog.uleg[3]; break;
    case LittleDog::B_FL_LLEG: b = mLittleDog.lleg[0]; break;
    case LittleDog::B_FR_LLEG: b = mLittleDog.lleg[1]; break;
    case LittleDog::B_HL_LLEG: b = mLittleDog.lleg[2]; break;
    case LittleDog::B_HR_LLEG: b = mLittleDog.lleg[3]; break;
    default: return;
  }
  
  odeOri = dBodyGetQuaternion(b);
  odePos = dBodyGetPosition(b);
  
  pos->n[0] = odePos[0];
  pos->n[1] = odePos[1];
  pos->n[2] = odePos[2];
  
  ori->n[3] = odeOri[0];
  ori->n[0] = odeOri[1];
  ori->n[1] = odeOri[2];
  ori->n[2] = odeOri[3];
}

// draw the simulation using the drawstuff library
void SDSimulator::dsDraw()
{
  dsSetColor(0.8,0.8,0.8);
  SDDrawGeom(mLittleDog.trunkGeom);

  dQuaternion headPoint, temp;
  headPoint[0] = 0.0;
  headPoint[1] = mSdc.mLDTrunkSizeX / 2.0;
  headPoint[2] = 0.0;
  headPoint[3] = 0.0;
  dQMultiply0(temp, dBodyGetQuaternion(mLittleDog.trunk), headPoint);
  dQMultiply2(headPoint, temp, dBodyGetQuaternion(mLittleDog.trunk));
  dsSetColor(0.9, 0.9, 0.2);
  headPoint[1] += dBodyGetPosition(mLittleDog.trunk)[0];
  headPoint[2] += dBodyGetPosition(mLittleDog.trunk)[1];
  headPoint[3] += dBodyGetPosition(mLittleDog.trunk)[2];
  SDDrawPoint(&headPoint[1], 0.02);
    
  for (int i = 0; i < NUM_LEGS; i++) {
    dsSetColor(0.5, 0.5, 1.0);
      
    SDDrawGeom(mLittleDog.hipGeom[i]);
    SDDrawGeom(mLittleDog.ulegGeom[i]);
    SDDrawGeom(mLittleDog.llegGeom[i]);
  }

  if (mTerrain != 0) {
    SDDrawGeom(mTerrain->getTrimeshGeom());
  }
}

// Set the mean and variance of the servo gain
void SDSimulator::setServoGain(double servoGainMean, double servoGainSig)
{
  mOde.servoGainMean = servoGainMean;
  mOde.servoGainSig = servoGainSig;
}

// Set the mean and variance of the friction
void SDSimulator::setFriction(double frictionMean, double frictionSig)
{
  mOde.frictionMean = frictionMean;
  mOde.frictionSig = frictionSig;
}

// load a series of files for playback
void SDSimulator::setPlaybackFile(char* filename)
{
  strncpy(mPlaybackFile, filename, 256);
  mPlaybackIndex = 0;
  loadCurrentPlaybackFrame();
  mPlayback = true;
}

// write the current state of the simulator to a file
void SDSimulator::writeStateToFile(char* filename)
{
  ofstream fout;

  fout.open(filename);
  if (!fout.good()) {
    SDPrintErr("Couldn't open file in writeStateToFile()");
    return;
  }

  fout << mTimestamp << endl;
  writeBodyInfo(mLittleDog.trunk, fout);
  for (int i = 0; i < NUM_LEGS; i++) {
    writeBodyInfo(mLittleDog.hip[i], fout);
    writeBodyInfo(mLittleDog.uleg[i], fout);
    writeBodyInfo(mLittleDog.lleg[i], fout);
  }
  fout.close();
}

// read state from a file
void SDSimulator::loadStateFromFile(char* filename)
{
  ifstream fin;

  fin.open(filename);
  if (!fin.good()) {
    SDPrintErr("Couldn't open file in loadStateFromFile()");
    return;
  }

  fin >> mTimestamp;
  readBodyInfo(mLittleDog.trunk, fin);
  for (int i = 0; i < NUM_LEGS; i++) {
    readBodyInfo(mLittleDog.hip[i], fin);
    readBodyInfo(mLittleDog.uleg[i], fin);
    readBodyInfo(mLittleDog.lleg[i], fin);
  }
  fin.close();
}


// get the current state of the simulator
void SDSimulator::getState(SDSimulatorState *state)
{
  state->timestamp = mTimestamp;
  getBodyState(mLittleDog.trunk, state->bodyStates[0]);
  for (int i = 0; i < NUM_LEGS; i++) {
    getBodyState(mLittleDog.hip[i], state->bodyStates[i*3+1]);
    getBodyState(mLittleDog.uleg[i], state->bodyStates[i*3+2]);
    getBodyState(mLittleDog.lleg[i], state->bodyStates[i*3+3]);
  }

  for (int i = 0; i < NUM_LEGS; i++) {
    state->jointVelocities[i*3] = dJointGetHingeParam(mLittleDog.hipRx[i],
						      dParamVel);
    state->jointVelocities[i*3+1] = dJointGetHingeParam(mLittleDog.hipRy[i],
							dParamVel);
    state->jointVelocities[i*3+2] = dJointGetHingeParam(mLittleDog.kneeRy[i],
							dParamVel);
  }
}


// set the current state
void SDSimulator::setState(const SDSimulatorState *state)
{
  mTimestamp = state->timestamp;
  setBodyState(mLittleDog.trunk, state->bodyStates[0]);
  for (int i = 0; i < NUM_LEGS; i++) {
    setBodyState(mLittleDog.hip[i], state->bodyStates[i*3+1]);
    setBodyState(mLittleDog.uleg[i], state->bodyStates[i*3+2]);
    setBodyState(mLittleDog.lleg[i], state->bodyStates[i*3+3]);
  }

  for (int i = 0; i < NUM_LEGS; i++) {
    dJointSetHingeParam(mLittleDog.hipRx[i], dParamVel,
			state->jointVelocities[i*3]);
    dJointSetHingeParam(mLittleDog.hipRy[i], dParamVel,
			state->jointVelocities[i*3+1]);
    dJointSetHingeParam(mLittleDog.kneeRy[i], dParamVel,
			state->jointVelocities[i*3+2]);
  }
}

// get the local position of the robot feet
void SDSimulator::getLocalFootPositions(bduVec3f *localFeet)
{
  bduVec3f angles[4];
  getDogAngles(angles);
  for (int i = 0; i < NUM_LEGS; i++) {
    SDFwdKin(i, angles[i], &localFeet[i]);
  }
}

// get the global position of the robot feet
void SDSimulator::getGlobalFootPositions(bduVec3f *globalFeet)
{
  bduVec3f angles[4], localFeet[4];
  bduVec3f pos, ori;
  bduVec4f qOri;
  
  getDogAngles(angles);
  getBodyInformation(LittleDog::B_TRUNK, &pos, &qOri);
  SDQuaternionToEuler(qOri, &ori);

  for (int i = 0; i < NUM_LEGS; i++) {
    SDFwdKin(i, angles[i], &localFeet[i]);
    SDLocalToWorld(pos, ori, localFeet[i], &globalFeet[i]);
  }
}

// get the local position of a step
void SDSimulator::getLocalStepPosition(double x, double y, bduVec3f *localStep)
{
  bduVec3f globalStep;
  bduVec3f pos, ori;
  bduVec4f qOri;

  globalStep.n[0] = x;
  globalStep.n[1] = y;
  globalStep.n[2] = getPointHeight(x,y);
  
  getBodyInformation(LittleDog::B_TRUNK, &pos, &qOri);
  SDQuaternionToEuler(qOri, &ori);
  SDWorldToLocal(pos, ori, globalStep, localStep);
}




// get the state of a single body
void SDSimulator::getBodyState(dBodyID body, double *state)
{
  const dReal *d;
  d = dBodyGetPosition(body);
  state[0] = d[0];
  state[1] = d[1];
  state[2] = d[2];
  d = dBodyGetQuaternion(body);
  state[3] = d[0];
  state[4] = d[1];
  state[5] = d[2];
  state[6] = d[3];
  d = dBodyGetLinearVel(body);
  state[7] = d[0];
  state[8] = d[1];
  state[9] = d[2];
  d = dBodyGetAngularVel(body);
  state[10] = d[0];
  state[11] = d[1];
  state[12] = d[2];
}

// set the state of a single body
void SDSimulator::setBodyState(dBodyID body, const double *state)
{
  dQuaternion q;

  dBodySetPosition(body, state[0], state[1], state[2]);
  q[0] = state[3];
  q[1] = state[4];
  q[2] = state[5];
  q[3] = state[6];
  dBodySetQuaternion(body, q);
  dBodySetLinearVel(body, state[7], state[8], state[9]);
  dBodySetAngularVel(body, state[10], state[11], state[12]);
}


// write a body's information to a file
void SDSimulator::writeBodyInfo(dBodyID body, ofstream &fout)
{
  const dReal *d;

  d = dBodyGetPosition(body);
  fout << d[0] << " " << d[1] << " " << d[2] << endl;
  d = dBodyGetQuaternion(body);
  fout << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << endl;
  d = dBodyGetLinearVel(body);
  fout << d[0] << " " << d[1] << " " << d[2] << endl;
  d = dBodyGetAngularVel(body);
  fout << d[0] << " " << d[1] << " " << d[2] << endl;
}

// read a body's information from a file
void SDSimulator::readBodyInfo(dBodyID body, ifstream &fin)
{
  dVector3 d;
  dQuaternion q;

  fin >> d[0] >> d[1] >> d[2];
  dBodySetPosition(body, d[0], d[1], d[2]);
  fin >> q[0] >> q[1] >> q[2] >> q[3];
  dBodySetQuaternion(body, q);
  fin >> d[0] >> d[1] >> d[2];
  dBodySetLinearVel(body, d[0], d[1], d[2]);
  fin >> d[0] >> d[1] >> d[2];
  dBodySetAngularVel(body, d[0], d[1], d[2]);
}

// load one frame from the playback
void SDSimulator::loadCurrentPlaybackFrame()
{
  char filename[256];
  sprintf(filename, "%s.%d", mPlaybackFile, mPlaybackIndex);
  cout << "Trying to open " << filename << endl;
  loadStateFromFile(filename);
}
  

// Build the Little Dog model in the world
void SDSimulator::buildLittleDogModel()
{
  dVector3 trunkPos, hipPos[4], ulegPos[4], llegPos[4];
  dVector3 hipRxPos[4], hipRyPos[4], kneeRyPos[4], footPos[4];
  dVector3 hipCom[4], ulegCom[4], llegCom[4];
  dMass m;

  trunkPos[0] = mSdc.mLDBuildPosX;
  trunkPos[1] = mSdc.mLDBuildPosY;
  trunkPos[2] = mSdc.mLDBuildPosZ;

  // Find joint positions
  for (int i = 0; i < 4; i++) {
    double xSign = (i < 2 ? 1.0 : -1.0);
    double ySign = (i % 2 == 0 ? 1.0 : -1.0);
    
    hipRxPos[i][0] = trunkPos[0] + xSign * mSdc.mLDHipRxOffsetX;
    hipRxPos[i][1] = trunkPos[1] + ySign * mSdc.mLDHipRxOffsetY;
    hipRxPos[i][2] = trunkPos[2] + mSdc.mLDHipRxOffsetZ;

    hipRyPos[i][0] = hipRxPos[i][0] + xSign * mSdc.mLDHipRyOffsetX;
    hipRyPos[i][1] = hipRxPos[i][1] + ySign * mSdc.mLDHipRyOffsetY;
    hipRyPos[i][2] = hipRxPos[i][2] + mSdc.mLDHipRyOffsetZ;

    kneeRyPos[i][0] = hipRyPos[i][0] + xSign * mSdc.mLDKneeRyOffsetX;
    kneeRyPos[i][1] = hipRyPos[i][1] + ySign * mSdc.mLDKneeRyOffsetY;
    kneeRyPos[i][2] = hipRyPos[i][2] + mSdc.mLDKneeRyOffsetZ;

    footPos[i][0] = kneeRyPos[i][0] + xSign * mSdc.mLDFootOffsetX;
    footPos[i][1] = kneeRyPos[i][1] + ySign * mSdc.mLDFootOffsetY;
    footPos[i][2] = kneeRyPos[i][2] + mSdc.mLDFootOffsetZ;
  }

  // calculate body positions and center of masses
  for (int i = 0; i < 4; i++) {
    double xSign = (i < 2 ? 1.0 : -1.0);
    double ySign = (i % 2 == 0 ? 1.0 : -1.0);
    
    // hip
    hipPos[i][0] = hipRxPos[i][0] + (hipRyPos[i][0] - hipRxPos[i][0]) / 2;
    hipPos[i][1] = hipRxPos[i][1] + (hipRyPos[i][1] - hipRxPos[i][1]) / 2;
    hipPos[i][2] = hipRxPos[i][2] + (hipRyPos[i][2] - hipRxPos[i][2]) / 2;
    
    hipCom[i][0] = hipRxPos[i][0] + xSign*mSdc.mLDHipCOMX - hipPos[i][0];
    hipCom[i][1] = hipRxPos[i][1] + ySign*mSdc.mLDHipCOMY - hipPos[i][1];
    hipCom[i][2] = hipRxPos[i][2] + mSdc.mLDHipCOMZ - hipPos[i][2];

    // upper leg
    ulegPos[i][0] = hipRyPos[i][0] + (kneeRyPos[i][0] - hipRyPos[i][0]) / 2;
    ulegPos[i][1] = hipRyPos[i][1] + (kneeRyPos[i][1] - hipRyPos[i][1]) / 2;
    ulegPos[i][2] = hipRyPos[i][2] + (kneeRyPos[i][2] - hipRyPos[i][2]) / 2;

    ulegCom[i][0] = hipRyPos[i][0] + xSign*mSdc.mLDULegCOMX - ulegPos[i][0];
    ulegCom[i][1] = hipRyPos[i][1] + ySign*mSdc.mLDULegCOMY - ulegPos[i][1];
    ulegCom[i][2] = hipRyPos[i][2] + mSdc.mLDULegCOMZ - ulegPos[i][2];

    // lower leg
    llegPos[i][0] = footPos[i][0];
    llegPos[i][1] = footPos[i][1];
    llegPos[i][2] = kneeRyPos[i][2] + (footPos[i][2] - kneeRyPos[i][2])/ 2;

    llegCom[i][0] = kneeRyPos[i][0] + xSign*mSdc.mLDLLegCOMX - llegPos[i][0];
    llegCom[i][1] = kneeRyPos[i][1] + ySign*mSdc.mLDLLegCOMY - llegPos[i][1];
    llegCom[i][2] = kneeRyPos[i][2] + mSdc.mLDLLegCOMZ - llegPos[i][2];
  }

  // create the ode bodies
  mLittleDog.trunkGeom = dCreateBox(mOde.littleDogSpace, mSdc.mLDTrunkSizeX,
				    mSdc.mLDTrunkSizeY, mSdc.mLDTrunkSizeZ);
  mLittleDog.trunk = dBodyCreate(mOde.world);
  dGeomSetBody(mLittleDog.trunkGeom, mLittleDog.trunk);
  dBodySetPosition(mLittleDog.trunk, trunkPos[0], trunkPos[1], trunkPos[2]);
  dMassSetBoxTotal (&m, mSdc.mLDTrunkMass, mSdc.mLDTrunkSizeX,
		    mSdc.mLDTrunkSizeY, mSdc.mLDTrunkSizeZ);
  dMassTranslate(&m, mSdc.mLDTrunkCOMX, mSdc.mLDTrunkCOMY,  mSdc.mLDTrunkCOMZ);
  dBodySetMass(mLittleDog.trunk, &m);

  for (int i = 0; i < 4; i++) {
    // hip
    mLittleDog.hipGeom[i] = dCreateBox(mOde.littleDogSpace, mSdc.mLDHipSizeX,
				       mSdc.mLDHipSizeY, mSdc.mLDHipSizeZ);
    mLittleDog.hip[i] = dBodyCreate(mOde.world);
    dGeomSetBody(mLittleDog.hipGeom[i], mLittleDog.hip[i]);
    dBodySetPosition(mLittleDog.hip[i], hipPos[i][0],
		     hipPos[i][1], hipPos[i][2]);
    dMassSetBoxTotal(&m, mSdc.mLDHipMass, mSdc.mLDHipSizeX,
		     mSdc.mLDHipSizeY, mSdc.mLDHipSizeZ);
    dMassTranslate(&m, hipCom[i][0], hipCom[i][1], hipCom[i][2]);
    dBodySetMass(mLittleDog.hip[i], &m);

    // upper leg
    mLittleDog.ulegGeom[i] = dCreateBox(mOde.littleDogSpace, mSdc.mLDULegSizeX,
					mSdc.mLDULegSizeY, mSdc.mLDULegSizeZ);
    mLittleDog.uleg[i] = dBodyCreate(mOde.world);
    dGeomSetBody(mLittleDog.ulegGeom[i], mLittleDog.uleg[i]);
    dBodySetPosition(mLittleDog.uleg[i], ulegPos[i][0],
		     ulegPos[i][1], ulegPos[i][2]);
    dMassSetBoxTotal(&m, mSdc.mLDULegMass, mSdc.mLDULegSizeX,
		     mSdc.mLDULegSizeY, mSdc.mLDULegSizeZ);
    dMassTranslate(&m, ulegCom[i][0], ulegCom[i][1], ulegCom[i][2]);
    dBodySetMass(mLittleDog.uleg[i], &m);

    // lower leg
    mLittleDog.llegGeom[i] =
      dCreateCCylinder(mOde.littleDogSpace, mSdc.mLDFootRadius,
		       mSdc.mLDLLegSizeZ- 2*mSdc.mLDFootRadius);
    mLittleDog.lleg[i] = dBodyCreate(mOde.world);
    dGeomSetBody(mLittleDog.llegGeom[i], mLittleDog.lleg[i]);
    dBodySetPosition(mLittleDog.lleg[i], llegPos[i][0],
		     llegPos[i][1], llegPos[i][2]);
    dMassSetCappedCylinderTotal(&m, mSdc.mLDLLegMass, 3,
				mSdc.mLDFootRadius,
				mSdc.mLDLLegSizeZ- 2*mSdc.mLDFootRadius);
    dMassTranslate(&m, llegCom[i][0], llegCom[i][1], llegCom[i][2]);
    dBodySetMass(mLittleDog.lleg[i], &m);
  }

  // create the joints
  for (int i = 0; i < 4; i++) {
    bool front = (i < 2);
    bool left = (i % 2 == 0);
    
    // hip rx joint
    mLittleDog.hipRx[i] = dJointCreateHinge(mOde.world, 0);
    dJointAttach(mLittleDog.hipRx[i], mLittleDog.trunk, mLittleDog.hip[i]);
    dJointSetHingeAnchor(mLittleDog.hipRx[i], hipRxPos[i][0],
			 hipRxPos[i][1], hipRxPos[i][2]);
    dJointSetHingeAxis(mLittleDog.hipRx[i], -1, 0, 0);
    dJointSetHingeParam(mLittleDog.hipRx[i], dParamFMax, mSdc.mLDHipMaxTorque);
    dJointSetHingeParam(mLittleDog.hipRx[i], dParamFudgeFactor,
			mSdc.mODEFudgeFactor);
    if (left) {
      dJointSetHingeParam(mLittleDog.hipRx[i], dParamLoStop,
			  mSdc.mLDHipRxStopLow);
      dJointSetHingeParam(mLittleDog.hipRx[i], dParamHiStop,
			  mSdc.mLDHipRxStopHigh);
    } else {
      dJointSetHingeParam(mLittleDog.hipRx[i], dParamLoStop,
			  -(mSdc.mLDHipRxStopHigh));
      dJointSetHingeParam(mLittleDog.hipRx[i], dParamHiStop,
			  -(mSdc.mLDHipRxStopLow));
    }

    
    // hip ry joint
    mLittleDog.hipRy[i] = dJointCreateHinge(mOde.world, 0);
    dJointAttach(mLittleDog.hipRy[i], mLittleDog.hip[i], mLittleDog.uleg[i]);
    dJointSetHingeAnchor(mLittleDog.hipRy[i], hipRyPos[i][0],
			 hipRyPos[i][1], hipRyPos[i][2]);
    dJointSetHingeAxis(mLittleDog.hipRy[i], 0, -1, 0);
    dJointSetHingeParam(mLittleDog.hipRy[i], dParamFMax, mSdc.mLDHipMaxTorque);
    dJointSetHingeParam(mLittleDog.hipRy[i], dParamFudgeFactor,
			mSdc.mODEFudgeFactor);
    if (front) {
      dJointSetHingeParam(mLittleDog.hipRy[i], dParamLoStop,
			  mSdc.mLDHipRyStopLow);
      dJointSetHingeParam(mLittleDog.hipRy[i], dParamHiStop,
			  mSdc.mLDHipRyStopHigh);
    } else {
      dJointSetHingeParam(mLittleDog.hipRy[i], dParamHiStop,
			  -(mSdc.mLDHipRyStopLow));
      dJointSetHingeParam(mLittleDog.hipRy[i], dParamLoStop,
			  -(mSdc.mLDHipRyStopHigh));
    }

    // knee Ry joint
    mLittleDog.kneeRy[i] = dJointCreateHinge(mOde.world, 0);
    dJointAttach(mLittleDog.kneeRy[i], mLittleDog.uleg[i], mLittleDog.lleg[i]);
    dJointSetHingeAnchor(mLittleDog.kneeRy[i], kneeRyPos[i][0],
			 kneeRyPos[i][1], kneeRyPos[i][2]);
    dJointSetHingeAxis(mLittleDog.kneeRy[i], 0, -1, 0);
    dJointSetHingeParam(mLittleDog.kneeRy[i], dParamFMax, mSdc.mLDKneeMaxTorque);
    dJointSetHingeParam(mLittleDog.kneeRy[i], dParamFudgeFactor,
			mSdc.mODEFudgeFactor);
    if (front) {
      dJointSetHingeParam(mLittleDog.kneeRy[i], dParamLoStop,
			  mSdc.mLDKneeRyStopLow);
      dJointSetHingeParam(mLittleDog.kneeRy[i], dParamHiStop,
			  mSdc.mLDKneeRyStopHigh);
    } else {
      dJointSetHingeParam(mLittleDog.kneeRy[i], dParamHiStop,
			  -(mSdc.mLDKneeRyStopLow));
      dJointSetHingeParam(mLittleDog.kneeRy[i], dParamLoStop,
			  -(mSdc.mLDKneeRyStopHigh));
    }	  
  }
}
