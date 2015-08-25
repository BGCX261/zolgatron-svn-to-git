#ifndef __SD_CONTROLLER_H__
#define __SD_CONTROLLER_H__

#include "SDLittleDog.h"

class SDController
{
 public:
  SDController() {;}
  virtual ~SDController() {;}
  
  virtual void getAngles(bduVec3f *angles) = 0;
  virtual void advanceAngles() = 0;
  virtual bool reset() = 0;
  virtual void dsDraw() {;}

  // accessors and observers
  void setLittleDog(SDLittleDog *littleDog)
    { mLittleDog = littleDog; }

 protected:
  SDLittleDog *mLittleDog;
};

#endif
