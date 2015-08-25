#ifndef __SD_KINEMATICS_H__
#define __SD_KINEMATICS_H__

#include <bduVec3f.h>

bool SDInvKin(int leg, bduVec3f& pos, bduVec3f *qForward, bduVec3f *qBackward);
void SDFwdKin(int leg, const bduVec3f& angles, bduVec3f *pos);
void SDFullFwdKin(int leg, const bduVec3f& angles, bduVec3f* hipRx,
		  bduVec3f *hipRy, bduVec3f *knee, bduVec3f *foot,
		  bduVec3f *hipRxAxis, bduVec3f *hipRyAxis,bduVec3f *kneeAxis);
bool SDFwdInvKin(int leg, double kneeAngle, const bduVec3f &p1,
		 const bduVec3f &p2, bduVec3f *angles, double *t);


#endif
