/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __SD_CONSTANTS_H__
#define __SD_CONSTANTS_H__

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <string>
#include <fstream>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const int NUM_LEGS = 4;
const int NUM_JOINTS = 3;
const int MAX_TERRAINS = 3;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class SDConstants
{

 public:

  static const SDConstants& getInstance();
		
  void display() const;

  double mODEWorldCFM;
  double mODEWorldERP;
  double mODEContactCFM;
  double mODEContactERP;
  double mODEFudgeFactor;

  double mODEWorldStep;
  double mODEFriction;
  double mODESlipFactor1;
  double mODESlipFactor2;
  double mODEServoGain;

  int mODEContactPoints;
  int mODERayContactPoints;

  double mLDBuildPosX;
  double mLDBuildPosY;
  double mLDBuildPosZ;

  double mLDTrunkSizeX;
  double mLDTrunkSizeY;
  double mLDTrunkSizeZ;

  double mLDHipSizeX;
  double mLDHipSizeY;
  double mLDHipSizeZ;

  double mLDULegSizeX;
  double mLDULegSizeY;
  double mLDULegSizeZ;

  double mLDLLegSizeZ;

  double mLDFootRadius;

  double mLDHipRxOffsetX;
  double mLDHipRxOffsetY;
  double mLDHipRxOffsetZ;

  double mLDHipRyOffsetX;
  double mLDHipRyOffsetY;
  double mLDHipRyOffsetZ;

  double mLDKneeRyOffsetX;
  double mLDKneeRyOffsetY;
  double mLDKneeRyOffsetZ;

  double mLDFootOffsetX;
  double mLDFootOffsetY;
  double mLDFootOffsetZ;

  double mLDHipMaxTorque;
  double mLDKneeMaxTorque;

  double mLDHipRxStopLow;
  double mLDHipRxStopHigh;

  double mLDHipRyStopLow;
  double mLDHipRyStopHigh;

  double mLDKneeRyStopLow;
  double mLDKneeRyStopHigh;

  double mLDTotalMass;
  double mLDTrunkMass;

  double mLDTrunkCOMX;
  double mLDTrunkCOMY;
  double mLDTrunkCOMZ;

  double mLDTrunkMOIXX;
  double mLDTrunkMOIYY;
  double mLDTrunkMOIZZ;

  double mLDHipMass;

  double mLDHipCOMX;
  double mLDHipCOMY;
  double mLDHipCOMZ;

  double mLDHipMOIXX;
  double mLDHipMOIYY;
  double mLDHipMOIZZ;

  double mLDULegMass;
  double mLDULegCOMX;
  double mLDULegCOMY;
  double mLDULegCOMZ;

  double mLDULegMOIXX;
  double mLDULegMOIYY;
  double mLDULegMOIZZ;

  double mLDLLegMass;
  double mLDLLegCOMX;
  double mLDLLegCOMY;
  double mLDLLegCOMZ;

  double mLDLLegMOIXX;
  double mLDLLegMOIYY;
  double mLDLLegMOIZZ;

  double mJointAngleOffsets[12];

 private:

  SDConstants();

  void readFromFile(std::string file);
  void read(int& target, std::ifstream& ifs);
  void read(double& target, std::ifstream& ifs);	

};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#endif
