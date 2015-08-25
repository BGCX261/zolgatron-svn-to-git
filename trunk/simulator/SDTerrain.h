#ifndef __SD_TERRAIN_H__
#define __SD_TERRAIN_H__

#include <vector>
#include <utility>
#include <ode/ode.h>

#define SAMPLE_POINTS 101

using namespace std;

class SDTerrain
{
 public:
  SDTerrain(char* filename);
  void buildInSpace(dSpaceID space);
  void setPosition(const dVector3 pos);
  void setOrientation(const dQuaternion orient);
  void dsDraw();
  void getPath(double goalX, double goalY, vector<pair<double,double> >&anchor);
  double distToGoal(int idx, double x, double y );
  double faceCost( int idx);
  double diffLevel();
  dGeomID getTrimeshGeom() { return mTrimeshGeom; }

  //private:
  // trimesh info
  int mNumVertices, mNumFaces;
  double *mVertices;
  int *mFaces;
  double *mNormals;
	int list_id;

  dTriMeshDataID mTrimeshData;
  dGeomID mTrimeshGeom;

  void calculateNormals();
};

#endif

