#include <fstream>
#include <algorithm>
#include <queue>
#include <iostream>
#include "SDTerrain.h"
#include "SDUtils.h"
#include <GL/glut.h>

using namespace std;

// load a terrain from a file
SDTerrain::SDTerrain(char* filename)
{
	list_id = -1;
  ifstream fin(filename);
  double scale;

  // read the trimesh directly
  fin >> mNumVertices >> mNumFaces >> scale;
  
  mVertices = new double [mNumVertices * 3];
  mFaces = new int [mNumFaces * 3];
  mNormals = 0;
  
  for (int i = 0; i < mNumVertices * 3; i++) {
    fin >> mVertices[i];
    mVertices[i] *= scale;
  }
  
  for (int i = 0; i < mNumFaces * 3; i++) {
    fin >> mFaces[i];
  }
 
  calculateNormals();
}

// build the trimesh in ODE
void SDTerrain::buildInSpace(dSpaceID space)
{
  mTrimeshData = dGeomTriMeshDataCreate();

  dGeomTriMeshDataBuildDouble1(mTrimeshData, mVertices, 3 * sizeof(double),
			       mNumVertices, mFaces, mNumFaces * 3,
			       3 * sizeof(int), mNormals);
  
  mTrimeshGeom = dCreateTriMesh(space, mTrimeshData, 0, 0, 0);
  dGeomSetData(mTrimeshGeom, (void*)this);
}

// position the terrain object
void SDTerrain::setPosition(const dVector3 pos)
{
  dGeomSetPosition(mTrimeshGeom, pos[0], pos[1], pos[2]);
}

// orient the terrain object
void SDTerrain::setOrientation(const dQuaternion orient)
{
  dGeomSetQuaternion(mTrimeshGeom, orient);
}

// draw the trimesh with drawstuff routines
void SDTerrain::dsDraw()
{
  const dReal *pos = dGeomGetPosition(mTrimeshGeom);
  const dReal *rot = dGeomGetRotation(mTrimeshGeom);

#if 0
	for (int i = 0; i < mNumFaces; i++) {
	  dsSetColor(0.7, 0.4, 0.2);
	  dsDrawTriangleD(pos, rot, &mVertices[mFaces[i * 3 + 0] * 3],
			  &mVertices[mFaces[i * 3 + 1] * 3],
			  &mVertices[mFaces[i * 3 + 2] * 3], 1);
	}
#else
	if ( list_id < 0 ) {
		list_id = glGenLists(1);
		glNewList(list_id, GL_COMPILE_AND_EXECUTE);
		for (int i = 0; i < mNumFaces; i++) {
		  dsSetColor(0.7, 0.4, 0.2);
		  dsDrawTriangleD(pos, rot, &mVertices[mFaces[i * 3 + 0] * 3],
				  &mVertices[mFaces[i * 3 + 1] * 3],
				  &mVertices[mFaces[i * 3 + 2] * 3], 1);
		}
		glEndList();
	} else {
		glCallList(list_id);
	}
#endif
}


// calculate normals for all the faces
void SDTerrain::calculateNormals()
{
  if (mNormals != 0) {
    delete mNormals;
  }

  mNormals = new double [mNumFaces * 3];

  for (int i = 0; i < mNumFaces; i++) {
    double *v1, *v2, *v3;
    double v1v2[3], v1v3[3];
    double len;
    
    v1 = &mVertices[mFaces[i*3+0]*3];
    v2 = &mVertices[mFaces[i*3+1]*3];
    v3 = &mVertices[mFaces[i*3+2]*3];

    v1v2[0] = v2[0] - v1[0];
    v1v2[1] = v2[1] - v1[1];
    v1v2[2] = v2[2] - v1[2];
    v1v3[0] = v3[0] - v1[0];
    v1v3[1] = v3[1] - v1[1];
    v1v3[2] = v3[2] - v1[2];

    mNormals[i*3+0] = v1v2[1]*v1v3[2] - v1v2[2]*v1v3[1];
    mNormals[i*3+1] = v1v2[2]*v1v3[0] - v1v2[0]*v1v3[2];
    mNormals[i*3+2] = v1v2[0]*v1v3[1] - v1v2[1]*v1v3[0];

    len = sqrt(pow(mNormals[i*3+0], 2) + pow(mNormals[i*3+1], 2) +
	       pow(mNormals[i*3+2], 2));
    
    mNormals[i*3+0] /= len;
    mNormals[i*3+1] /= len;
    mNormals[i*3+2] /= len;
  }
}

struct node{
  unsigned int idx;
  double costVal;
  double faceCostVal;
  double heightCostVal;
  double heuristicVal;
  int pathSize;
  vector<int> path;
  double heuristic() const {
    return heuristicVal;
  }
  double cost() const {
    return costVal+faceCostVal+heightCostVal;
  }
};

// Comparator for priority queue nodes in A* search.
// Class T should define cost() and heuristic() methods.
template<class T>
struct AStarComparator : public binary_function<T, T, bool> {
  bool operator() (const T& a, const T& b) const {
    // Priority of a is less than that of b if overall cost of a is
    // greater than that of b.
    return (a.cost() + a.heuristic() >= b.cost() + b.heuristic());
  }
};


double SDTerrain::distToGoal(int idx, double x, double y )
{
  return sqrt( pow(mVertices[idx*3]-x,2)+pow(mVertices[idx*3+1]-y,2) );
}

void printNode(node n)
{
  cout << "idx" <<n.idx <<" path_size "<< n.pathSize<< endl;
  cout << "cost "<<n.costVal<<" faceCost "<<n.faceCostVal<<" heightCost "<<n.heightCostVal<<" heu "<< n.heuristicVal<< endl;
}

double SDTerrain::faceCost(int idx)
{
  double cumN = 0;
  double avgNormal=0;
  int count = 0;
  int row = idx / SAMPLE_POINTS;
  int col = idx % SAMPLE_POINTS;
  double n;
  for( int r=row-2; r<=row+1; ++r){
    if( r<0 || r>=SAMPLE_POINTS-1 )
      continue;
    for( int c=col-2; c<=col+1; ++c){
      if( c<0 || c>= SAMPLE_POINTS-1 )
	continue;
      int s = r*2*(SAMPLE_POINTS-1) + 2*c;
      for( int i=0; i<2; ++i){
	n = mNormals[3*(s+i)+2];
	count ++;
	cumN += n;
      }
    }
  }
  avgNormal = 1-cumN/count;
  if(avgNormal < 0.02 )
    return 0;
  else
    return avgNormal;
}

double SDTerrain::diffLevel()
{
  double sum = 0;
  for( int i=0; i<mNumFaces; ++i)
    sum += mNormals[3*i+2];
  return 1-sum/mNumFaces;

}

void SDTerrain::getPath(double goalX, double goalY,
				     vector<pair<double,double> > &anchor)
{
  calculateNormals();
  double level = diffLevel();
  double gridSize = mVertices[3]-mVertices[0];
  priority_queue<node, vector<node>,  AStarComparator<node> > queue;
  bool history [mNumVertices];
  memset(history, 0, mNumVertices);
  node first, ans;
  first.idx = 5050;
  first.pathSize = 1;
  first.costVal = 0;
  first.heightCostVal=0;
  first.path.push_back(5050);
  first.heuristicVal = distToGoal(first.idx, goalX, goalY);
  first.faceCostVal = faceCost(5050);
  //cout << first.heuristicVal << " position "<< mVertices[first.idx*3]<<","<<mVertices[first.idx*3+1] <<endl;
  queue.push(first);
  int count = 0;

  while ( !queue.empty() ){
    count++;
    node currNode = queue.top();
    queue.pop();
   
    if(history[currNode.idx] == 1){
      continue;
    }else{
      history[currNode.idx] = 1;
    }
    //if( count <10)
      printNode(currNode);
    // if reach goal
    if (currNode.heuristicVal < gridSize*0.5) {
      cout << "Found!!!!!!!!" << endl;
      ans = currNode;
      break;
    }
    int row = currNode.idx / SAMPLE_POINTS;
    int col = currNode.idx % SAMPLE_POINTS;
    double currHeight = mVertices[currNode.idx*3+2];
    // put sucessors to the fringe
    for( int i=-1; i<2; ++i){
      for( int j=-1; j<2; j++){
	if( i==0 && j==0 )
	  continue;
	if( (row==0 && i<0) || (row==SAMPLE_POINTS-1 && i>0) ||  
	    (col==0 && j<0) || (col==SAMPLE_POINTS-1 && j>0) )
	  continue;
	node newNode;

	int newIdx = currNode.idx+i*SAMPLE_POINTS+j;
	if( newIdx<0 || newIdx>=mNumVertices )
	  continue;
	newNode.idx = newIdx;
	newNode.heuristicVal =  distToGoal(newNode.idx, goalX,goalY);
	newNode.path = currNode.path;
	newNode.path.push_back( newIdx );
	newNode.pathSize = newNode.path.size();
	double heightCost = (currHeight-mVertices[3*newNode.idx+2])>0 ? 
	  (currHeight-mVertices[3*newNode.idx+2]) : -(currHeight-mVertices[3*newNode.idx+2]) ;
	double stepCost;
	if( i==0 || j==0 ){
	  stepCost = gridSize;
	}else{
	  stepCost = gridSize*sqrt(2);
	}
	if( heightCost>0.05 )
	  heightCost *= 50;
	else if( heightCost>0.03)
	  heightCost *= 20;  
	else if( heightCost>0.02)
	  heightCost *=10;
	newNode.heightCostVal = currNode.heightCostVal + heightCost;
	newNode.costVal = currNode.costVal + stepCost;
	newNode.faceCostVal = currNode.faceCostVal + faceCost(newIdx);
	queue.push(newNode);
      }
    }
  }
  printf("diffLevel %f\nstep size %d, length: %f\n", level, ans.path.size(), ans.costVal);
  // now store path in ans node to position anchor
  for( int i=0; i<ans.path.size(); ++i){
    anchor.push_back(make_pair(mVertices[3*ans.path[i]],mVertices[3*ans.path[i]+1]-1.5));
  }
}
