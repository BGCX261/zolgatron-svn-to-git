/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "SDConstants.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using std::string;
using std::ifstream;

#include <stdexcept>
using std::runtime_error;

#include <iostream>
using std::cout;
using std::endl;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const SDConstants& SDConstants::getInstance()
{
  static SDConstants sdc;
  static bool uninitialized = true;
	
  if ( uninitialized == true )
    {
      sdc.readFromFile("params/SDConstants.params");	
      uninitialized = false;
    }
	
  return sdc;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SDConstants::display() const
{

  cout << "Contents of SDConstants: " << endl << endl;	
	
  cout << "mODEWorldCFM: " << mODEWorldCFM << endl;
  cout << "mODEWorldERP: " << mODEWorldERP << endl;
  cout << "mODEContactCFM: " << mODEContactCFM << endl;
  cout << "mODEContactERP: " << mODEContactERP << endl;
  cout << "mODEFudgeFactor: " << mODEFudgeFactor << endl;
	
  cout << "mODEWorldStep: " << mODEWorldStep << endl;
  cout << "mODEFriction: " << mODEFriction << endl;
  cout << "mODESlipFactor1: " << mODESlipFactor1 << endl;
  cout << "mODESlipFactor2: " << mODESlipFactor2 << endl;
  cout << "mODEServoGain: " << mODEServoGain << endl;
	
  cout << "mODEContactPoints: " << mODEContactPoints << endl;
  cout << "mODERayContactPoints: " << mODERayContactPoints << endl;
	
  cout << "mLDBuildPosX: " << mLDBuildPosX << endl;
  cout << "mLDBuildPosY: " << mLDBuildPosY << endl;
  cout << "mLDBuildPosZ: " << mLDBuildPosZ << endl;
	
  cout << "mLDTrunkSizeX: " << mLDTrunkSizeX << endl;
  cout << "mLDTrunkSizeY: " << mLDTrunkSizeY << endl;
  cout << "mLDTrunkSizeZ: " << mLDTrunkSizeZ << endl;
	
  cout << "mLDHipSizeX: " << mLDHipSizeX << endl;
  cout << "mLDHipSizeY: " << mLDHipSizeY<< endl;
  cout << "mLDHipSizeZ: " << mLDHipSizeZ << endl;
	
  cout << "mLDULegSizeX: " << mLDULegSizeX << endl;
  cout << "mLDULegSizeY: " << mLDULegSizeY << endl;
  cout << "mLDULegSizeZ: " << mLDULegSizeZ << endl;
	
  cout << "mLDLLegSizeZ: " << mLDLLegSizeZ << endl;
	
  cout << "mLDFootRadius: " << mLDFootRadius << endl;
	
  cout << "mLDHipRxOffsetX: " << mLDHipRxOffsetX << endl;
  cout << "mLDHipRxOffsetY: " << mLDHipRxOffsetY << endl;
  cout << "mLDHipRxOffsetZ: " << mLDHipRxOffsetZ << endl;
	
  cout << "mLDHipRyOffsetX: " << mLDHipRyOffsetX << endl;
  cout << "mLDHipRyOffsetY: " << mLDHipRyOffsetY << endl;
  cout << "mLDHipRyOffsetZ: " << mLDHipRyOffsetZ << endl;
	
  cout << "mLDKneeRyOffsetX: " << mLDKneeRyOffsetX << endl;
  cout << "mLDKneeRyOffsetY: " << mLDKneeRyOffsetY << endl;
  cout << "mLDKneeRyOffsetZ: " << mLDKneeRyOffsetZ << endl;
	
  cout << "mLDFootOffsetX: " << mLDFootOffsetX << endl;
  cout << "mLDFootOffsetY: " << mLDFootOffsetY << endl;
  cout << "mLDFootOffsetZ: " << mLDFootOffsetZ << endl;
	
  cout << "mLDHipMaxTorque: " << mLDHipMaxTorque << endl;
  cout << "mLDKneeMaxTorque: " << mLDKneeMaxTorque << endl;
	
  cout << "mLDHipRxStopLow: " << mLDHipRxStopLow << endl;
  cout << "mLDHipRxStopHigh: " << mLDHipRxStopHigh << endl;
	
  cout << "mLDHipRyStopLow: " << mLDHipRyStopLow << endl;
  cout << "mLDHipRyStopHigh: " << mLDHipRyStopHigh << endl;
	
  cout << "mLDKneeRyStopLow: " << mLDKneeRyStopLow << endl;
  cout << "mLDKneeRyStopHigh: " << mLDKneeRyStopHigh << endl;
	
  cout << "mLDTotalMass: " << mLDTotalMass << endl;
  cout << "mLDTrunkMass: " << mLDTrunkMass << endl;
	
  cout << "mLDTrunkCOMX: " << mLDTrunkCOMX << endl;
  cout << "mLDTrunkCOMY: " << mLDTrunkCOMY << endl;
  cout << "mLDTrunkCOMZ: " << mLDTrunkCOMZ << endl;
	
  cout << "mLDTrunkMOIXX: " << mLDTrunkMOIXX << endl;
  cout << "mLDTrunkMOIYY: " << mLDTrunkMOIYY << endl;
  cout << "mLDTrunkMOIZZ: " << mLDTrunkMOIZZ << endl;
	
  cout << "mLDHipMass: " << mLDHipMass << endl;
	
  cout << "mLDHipCOMX: " << mLDHipCOMX << endl;
  cout << "mLDHipCOMY: " << mLDHipCOMY << endl;
  cout << "mLDHipCOMZ: " << mLDHipCOMZ << endl;
	
  cout << "mLDHipMOIXX: " << mLDHipMOIXX << endl;
  cout << "mLDHipMOIYY: " << mLDHipMOIYY << endl;
  cout << "mLDHipMOIZZ: " << mLDHipMOIZZ << endl;
	
  cout << "mLDLegMass: " << mLDULegMass << endl;
  cout << "mLDLegCOMX: " << mLDULegCOMX << endl;
  cout << "mLDLegCOMY: " << mLDULegCOMY << endl;
  cout << "mLDLegCOMZ: " << mLDULegCOMZ << endl;
	
  cout << "mLDULegMOIXX: " << mLDULegMOIXX << endl;
  cout << "mLDULegMOIYY: " << mLDULegMOIYY << endl;
  cout << "mLDULegMOIZZ: " << mLDULegMOIZZ << endl;
	
  cout << "mLDLLegMass: " << mLDLLegMass << endl;
  cout << "mLDLLegCOMX: " << mLDLLegCOMX << endl;
  cout << "mLDLLegCOMY: " << mLDLLegCOMY << endl;
  cout << "mLDLLegCOMZ: " << mLDLLegCOMZ << endl;
	
  cout << "mLDLLegMOIXX: " << mLDLLegMOIXX << endl;
  cout << "mLDLLegMOIYY: " << mLDLLegMOIYY << endl;
  cout << "mLDLLegMOIZZ: " << mLDLLegMOIZZ << endl;

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

SDConstants::SDConstants() {}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SDConstants::readFromFile(string file)
{

  ifstream ifs(file.c_str());
  if ( !ifs.good() )
    {
      string error = "SDConstants couldn't open file " + file;
      throw runtime_error(error);
    }

  read(mODEWorldCFM, ifs);
  read(mODEWorldERP, ifs);
  read(mODEContactCFM, ifs);
  read(mODEContactERP, ifs);
  read(mODEFudgeFactor, ifs);
	
  read(mODEWorldStep, ifs);
  read(mODEFriction, ifs);
  read(mODESlipFactor1, ifs);
  read(mODESlipFactor2, ifs);
  read(mODEServoGain, ifs);
	
  read(mODEContactPoints, ifs);
  read(mODERayContactPoints, ifs);
	
  read(mLDBuildPosX, ifs);
  read(mLDBuildPosY, ifs);
  read(mLDBuildPosZ, ifs);
	
  read(mLDTrunkSizeX, ifs);
  read(mLDTrunkSizeY, ifs);
  read(mLDTrunkSizeZ, ifs);
	
  read(mLDHipSizeX, ifs);
  read(mLDHipSizeY, ifs);
  read(mLDHipSizeZ, ifs);
	
  read(mLDULegSizeX, ifs);
  read(mLDULegSizeY, ifs);
  read(mLDULegSizeZ, ifs);
	
  read(mLDLLegSizeZ, ifs);
	
  read(mLDFootRadius, ifs);
	
  read(mLDHipRxOffsetX, ifs);
  read(mLDHipRxOffsetY, ifs);
  read(mLDHipRxOffsetZ, ifs);
	
  read(mLDHipRyOffsetX, ifs);
  read(mLDHipRyOffsetY, ifs);
  read(mLDHipRyOffsetZ, ifs);
	
  read(mLDKneeRyOffsetX, ifs);
  read(mLDKneeRyOffsetY, ifs);
  read(mLDKneeRyOffsetZ, ifs);
	
  read(mLDFootOffsetX, ifs);
  read(mLDFootOffsetY, ifs);
  read(mLDFootOffsetZ, ifs);
	
  read(mLDHipMaxTorque, ifs);
  read(mLDKneeMaxTorque, ifs);
	
  read(mLDHipRxStopLow, ifs);
  read(mLDHipRxStopHigh, ifs);
	
  read(mLDHipRyStopLow, ifs);
  read(mLDHipRyStopHigh, ifs);
	
  read(mLDKneeRyStopLow, ifs);
  read(mLDKneeRyStopHigh, ifs);
	
  read(mLDTotalMass, ifs);
  read(mLDTrunkMass, ifs);
	
  read(mLDTrunkCOMX, ifs);
  read(mLDTrunkCOMY, ifs);
  read(mLDTrunkCOMZ, ifs);
	
  read(mLDTrunkMOIXX, ifs);
  read(mLDTrunkMOIYY, ifs);
  read(mLDTrunkMOIZZ, ifs);
	
  read(mLDHipMass, ifs);
	
  read(mLDHipCOMX, ifs);
  read(mLDHipCOMY, ifs);
  read(mLDHipCOMZ, ifs);
	
  read(mLDHipMOIXX, ifs);
  read(mLDHipMOIYY, ifs);
  read(mLDHipMOIZZ, ifs);
	
  read(mLDULegMass, ifs);
  read(mLDULegCOMX, ifs);
  read(mLDULegCOMY, ifs);
  read(mLDULegCOMZ, ifs);
	
  read(mLDULegMOIXX, ifs);
  read(mLDULegMOIYY, ifs);
  read(mLDULegMOIZZ, ifs);
	
  read(mLDLLegMass, ifs);
  read(mLDLLegCOMX, ifs);
  read(mLDLLegCOMY, ifs);
  read(mLDLLegCOMZ, ifs);
	
  read(mLDLLegMOIXX, ifs);
  read(mLDLLegMOIYY, ifs);
  read(mLDLLegMOIZZ, ifs);

  for (int i = 0; i < 12; i++) {
    read(mJointAngleOffsets[i], ifs);
  }

  ifs.close();
  
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SDConstants::read(int& target, ifstream& ifs)
{
  ifs >> target;
  if ( !ifs.good() )
    {
      throw runtime_error("SDConstant couldn't read value from file");
    }
  while ( ifs.get() != '\n' );
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SDConstants::read(double& target, ifstream& ifs)
{
  ifs >> target;
  if ( !ifs.good() )
    {
      throw runtime_error("SDConstant couldn't read value from file");
    }
  while ( ifs.get() != '\n' );
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
