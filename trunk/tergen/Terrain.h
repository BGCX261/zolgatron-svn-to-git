#ifndef __TERRAIN_H__
#define __TERRAIN_H__

#include <ode/ode.h>

class Terrain
{
 private:
  // heightmap info
  double xRes, yRes;
  int xSize, ySize;
  double **heights;

  // trimesh info
  int numVertices, numFaces;
  double *vertices;
  int *faces;
  double *normals;
  bool *nearFaces;

  dTriMeshDataID trimeshData;
  dGeomID trimeshGeom;

  void calculateNormals();

 public:
  Terrain(char* filename, bool heightmap);
  void writeAsTrimesh(char* filename);
  void buildInSpace(dSpaceID space);
  void setPosition(const dVector3 pos);
  void setOrientation(const dQuaternion orient);
  void dsDraw();

  // distance functions
  void setNearFaces(const int* triIndices, int triCount);
  void trianglesNearPoint(dVector3 p, double maxDist);
  double distanceFromPointToFace(const dReal *p, int face);

  dGeomID getTrimeshGeom() { return trimeshGeom; }
};

#endif
