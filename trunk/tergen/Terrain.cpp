#include <iostream>
#include <fstream>
#include <drawstuff/drawstuff.h>
#include "Terrain.h"
#include "VectorOp.h"

using namespace std;

// called when testing trimesh collison
void triArrayCallback(dGeomID triMesh, dGeomID refObject,
		      const int* triIndices, int triCount)
{
  const dReal *refPos, *trimeshPos;
  dVector3 offsetPos;
  dVector3 relPos;
  
  Terrain *terrain = (Terrain*)dGeomGetData(triMesh);
  terrain->setNearFaces(triIndices, triCount);

  refPos = dGeomGetPosition(refObject);
  trimeshPos = dGeomGetPosition(triMesh);
  
  offsetPos[0] = refPos[0] - trimeshPos[0];
  offsetPos[1] = refPos[1] - trimeshPos[1];
  offsetPos[2] = refPos[2] - trimeshPos[2];
  offsetPos[3] = 0;
  dMULTIPLY0_331(relPos, dGeomGetRotation(triMesh), offsetPos);

  cout << relPos[0] << " " << relPos[1] << " " << relPos[2] << endl;
  
  for (int i = 0; i < triCount; i++) {
    cout << triIndices[i] << ": ";
    cout << terrain->distanceFromPointToFace(relPos, triIndices[i]) << endl;
  }
}

// load a terrain from a file
Terrain::Terrain(char* filename, bool heightmap)
{
  ifstream fin(filename);
  double scale;

  if (heightmap) {
    fin >> xSize >> ySize >> xRes >> yRes >> scale;

    xRes *= scale;
    yRes *= scale;

    // initialize the heights array
    heights = new double* [ySize];
    for (int i = 0; i < ySize; i++) {
      heights[i] = new double [xSize];
      for (int j = 0; j < xSize; j++) {
	fin >> heights[i][j];
	heights[i][j] *= scale;
      }
    }

    // initialize the vertices
    numVertices = xSize * ySize;
    vertices = new double [numVertices * 3];
    for (int i = 0; i < ySize; i++) {
      for (int j = 0; j < xSize; j++) {
	vertices[(i * xSize + j) * 3 + 0] = j*xRes;
	vertices[(i * xSize + j) * 3 + 1] = i*yRes;
	vertices[(i * xSize + j) * 3 + 2] = heights[i][j];
      }
    }

    // initialize the faces
    numFaces = (xSize - 1) * (ySize - 1) * 2;
    faces = new int [numFaces * 3];
    for (int i = 0; i < ySize - 1; i++) {
      for (int j = 0; j < xSize - 1; j++) {
	faces[(i * (xSize - 1) + j) * 6] = i * xSize + j;
	faces[(i * (xSize - 1) + j) * 6 + 1] = i * xSize + (j+1);
	faces[(i * (xSize - 1) + j) * 6 + 2] = (i+1) * xSize + j;
	
	faces[(i * (xSize - 1) + j) * 6 + 3] = i * xSize + (j+1);
	faces[(i * (xSize - 1) + j) * 6 + 4] = (i+1) * xSize + (j+1);
	faces[(i * (xSize - 1) + j) * 6 + 5] = (i+1) * xSize + j;
      }
    }

    normals = 0;
  } else {
    // read the trimesh directly
    fin >> numVertices >> numFaces >> scale;
    //fin >> refRot[0] >> refRot[1] >> refRot[2] >> refRot[3];
    //fin >> refPos[0] >> refPos[1] >> refPos[2];
    
    vertices = new double [numVertices * 3];
    faces = new int [numFaces * 3];
    nearFaces = new bool [numFaces];
    normals = 0;

    for (int i = 0; i < numVertices * 3; i++) {
      fin >> vertices[i];
      vertices[i] *= scale;
    }

    for (int i = 0; i < numFaces * 3; i++) {
      fin >> faces[i];
    }

    for (int i = 0; i < numFaces; i++) {
      nearFaces[i] = false;
    }

    cout << numVertices << " " << numFaces << endl;
  }
  calculateNormals();
}

// Write a trimesh file
void Terrain::writeAsTrimesh(char* filename)
{
  ofstream fout;

  fout.open(filename);
  
  fout << numVertices << " " << numFaces << " 1.0" << endl;
  for (int i = 0; i < numVertices; i++) {
    fout << vertices[i*3] << " " << vertices[i*3+1] << " "
	 << vertices[i*3+2] << endl;
  }

  for (int i = 0;  i < numFaces; i++) {
    fout << faces[i*3] << " " << faces[i*3+1] << " " << faces[i*3+2] << endl;
  }
}
  
  

// build the trimesh in ODE
void Terrain::buildInSpace(dSpaceID space)
{
  trimeshData = dGeomTriMeshDataCreate();

  dGeomTriMeshDataBuildDouble1(trimeshData, vertices, 3 * sizeof(double),
			       numVertices, faces, numFaces * 3,
			       3 * sizeof(int), normals);
  
  trimeshGeom = dCreateTriMesh(space, trimeshData, 0, 0, 0);
  dGeomSetData(trimeshGeom, (void*)this);
}

// position the terrain object
void Terrain::setPosition(const dVector3 pos)
{
  dGeomSetPosition(trimeshGeom, pos[0], pos[1], pos[2]);
}

// orient the terrain object
void Terrain::setOrientation(const dQuaternion orient)
{
  dGeomSetQuaternion(trimeshGeom, orient);
}

// draw the trimesh with drawstuff routines
void Terrain::dsDraw()
{
  const dReal *pos = dGeomGetPosition(trimeshGeom);
  const dReal *rot = dGeomGetRotation(trimeshGeom);

  
  for (int i = 0; i < numFaces; i++) {
    /*
    dReal v[9] = {
      vertices[faces[i * 3 + 0] * 3 + 0],
      vertices[faces[i * 3 + 0] * 3 + 1],
      vertices[faces[i * 3 + 0] * 3 + 2],
      vertices[faces[i * 3 + 1] * 3 + 0],
      vertices[faces[i * 3 + 1] * 3 + 1],
      vertices[faces[i * 3 + 1] * 3 + 2],
      vertices[faces[i * 3 + 2] * 3 + 0],
      vertices[faces[i * 3 + 2] * 3 + 1],
      vertices[faces[i * 3 + 2] * 3 + 2]
    };
    */
    /*if (i == 1656) {
      dsSetColor(0.3, 0.3, 0.9);
      } else */if (nearFaces[i]) {
      dsSetColor(0.9, 0.9, 0.2);
    } else {
      dsSetColor(0.7, 0.4, 0.2);
    }
      
    dsDrawTriangleD(pos, rot, &vertices[faces[i * 3 + 0] * 3],
		    &vertices[faces[i * 3 + 1] * 3],
		    &vertices[faces[i * 3 + 2] * 3], 1);
  }
}


// flag faces as begin near the point specified by trianglesNearPoint
void Terrain::setNearFaces(const int* triIndices, int triCount)
{
  for (int i = 0; i < numFaces; i++) {
    nearFaces[i] = false;
  }
  for (int i = 0; i < triCount; i++) {
    nearFaces[triIndices[i]] = true;
  }
}

// list all the triangles in the mesh within maxDist of p
void Terrain::trianglesNearPoint(dVector3 p, double maxDist)
{
  dGeomID sphere;
  dContactGeom contact;

  sphere = dCreateSphere(0, maxDist);
  dGeomSetPosition(sphere, p[0], p[1], p[2]);
  
  dGeomTriMeshSetArrayCallback(trimeshGeom, &triArrayCallback);
  dCollide(sphere, trimeshGeom, 1, &contact, sizeof(dContactGeom));
  dGeomTriMeshSetArrayCallback(trimeshGeom, 0);
}


// calculate the distance from a point to a triangle face
double Terrain::distanceFromPointToFace(const dReal *p, int face)
{
  double *v1, *v2, *v3, *n;
  double v1v2[3], v1v3[3], v2v3[3], v2v1[3], v1p[3], v2p[3], v3p[3];
  double v1v2p[3], v1v3p[3], v2v3p[3];
  double cp1[3], cp2[3];
  double t, d1, d2, d3;
  
  v1 = &vertices[faces[face*3+0]*3];
  v2 = &vertices[faces[face*3+1]*3];
  v3 = &vertices[faces[face*3+2]*3];
  n = &normals[face*3];

  VECTOR_OP(v1v2, -, v2, v1);
  VECTOR_OP(v1v3, -, v3, v1);
  VECTOR_OP(v2v3, -, v3, v2);
  VECTOR_OP(v2v1, -, v1, v2);
  
  VECTOR_OP(v1p, -, p, v1);
  VECTOR_OP(v2p, -, p, v2);
  VECTOR_OP(v3p, -, p, v3);

  // first check if the (projected) point lines within the triangle
  // see http://www.blackpawn.com/texts/pointinpoly/default.html

  VECTOR_CROSS(cp1, v1v2, v1v3);
  VECTOR_CROSS(cp2, v1v2, v1p);
  d1 = VECTOR_DOT(cp1, cp2);

  VECTOR_CROSS(cp1, v1v3, v1v2);
  VECTOR_CROSS(cp2, v1v3, v1p);
  d2 = VECTOR_DOT(cp1, cp2);

  VECTOR_CROSS(cp1, v2v3, v2v1);
  VECTOR_CROSS(cp2, v2v3, v2p);
  d3 = VECTOR_DOT(cp1, cp2);

  cout << "<" << d1 << " " << d2 << " " << d3 << "> ";
  
  if (d1 >= 0 && d2 >= 0 && d3 >= 0) {
    // the point lies within the triangle
    cout << "+ ";
    return VECTOR_DOT(v1, n) - VECTOR_DOT(p, n);
  } else {
    // the point lie outside the triangle
    cout << "- (";
    
    // calculate the distance from p to each of the line segments
    t = VECTOR_DOT(v1v2, v1p) / VECTOR_DOT(v1v2, v1v2);
    cout << t << " ";
    if (t <= 0) {
      d1 = VECTOR_LENGTH(v1p);
    } else if (t >= 1) {
      d1 = VECTOR_LENGTH(v2p);
    } else {
      v1v2p[0] = p[0] - (v1[0] + t * v1v2[0]);
      v1v2p[1] = p[1] - (v1[1] + t * v1v2[1]);
      v1v2p[2] = p[2] - (v1[2] + t * v1v2[2]);
      d1 = VECTOR_LENGTH(v1v2p);
    }

    t = VECTOR_DOT(v1v3, v1p) / VECTOR_DOT(v1v3, v1v3);
    cout << t << " ";
    if (t <= 0) {
      d2 = VECTOR_LENGTH(v1p);
    } else if (t >= 1) {
      d2 = VECTOR_LENGTH(v3p);
    } else {
      v1v3p[0] = p[0] - (v1[0] + t * v1v3[0]);
      v1v3p[1] = p[1] - (v1[1] + t * v1v3[1]);
      v1v3p[2] = p[2] - (v1[2] + t * v1v3[2]);
      d2 = VECTOR_LENGTH(v1v3p);
    }

    t = VECTOR_DOT(v2v3, v2p) / VECTOR_DOT(v2v3, v2v3);
    cout << t << ") ";
    if (t <= 0) {
      d3 = VECTOR_LENGTH(v2p);
    } else if (t >= 1) {
      d3 = VECTOR_LENGTH(v3p);
    } else {
      v2v3p[0] = p[0] - (v2[0] + t * v2v3[0]);
      v2v3p[1] = p[1] - (v2[1] + t * v2v3[1]);
      v2v3p[2] = p[2] - (v2[2] + t * v2v3[2]);
      d3 = VECTOR_LENGTH(v2v3p);
    }

    //cout << "(" << d1 << " " << d2 << " " << d3 << ") ";
    return MIN(d1, MIN(d2, d3));
  }
}

// calculate normals for all the faces
void Terrain::calculateNormals()
{
  if (normals != 0) {
    delete normals;
  }

  normals = new double [numFaces * 3];

  for (int i = 0; i < numFaces; i++) {
    double *v1, *v2, *v3;
    double v1v2[3], v1v3[3];
    
    v1 = &vertices[faces[i*3+0]*3];
    v2 = &vertices[faces[i*3+1]*3];
    v3 = &vertices[faces[i*3+2]*3];

    VECTOR_OP(v1v2, -, v2, v1);
    VECTOR_OP(v1v3, -, v3, v1);
    VECTOR_CROSS((&normals[i*3]), v1v2, v1v3);
    VECTOR_NORMALIZE((&normals[i*3]));
  }
}
    
    
    
  
  
