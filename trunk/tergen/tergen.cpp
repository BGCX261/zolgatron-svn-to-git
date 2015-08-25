#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "Terrain.h"

using namespace std;


// some constants

#define DENSITY (5.0)		// density of all objects
#define MAX_CONTACTS 8		// maximum number of contact points per body
#define MAX_OBJS 500
#define NUM_LEVELS 4
#define TERRAIN_SIZE 3.0
#define SAMPLE_POINTS 101
#define RAY_CONTACTS 10

// dynamics and collision objects

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static int numBodies;
static dBodyID bodies[MAX_OBJS];
static dGeomID geoms[MAX_OBJS];
double heights[SAMPLE_POINTS][SAMPLE_POINTS];


static double levelSettings[NUM_LEVELS][7] = {
  {0, 0.2, 0.4, 0.2, 0.3, 0.01, 0.05},
  {20, 0.2, 0.4, 0.2, 0.3, 0.01, 0.2},
  {50, 0.2, 0.4, 0.2, 0.3, 0.01, 0.05},
  {100, 0.2, 0.4, 0.2, 0.3, 0.01, 0.05}
};
int level;


static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
    }
  }
}


void sampleHeightmap()
{
  dSpaceID raySpace;

  raySpace = dHashSpaceCreate(0);
  
  for (int i = 0; i < SAMPLE_POINTS; i++) {
    for (int j = 0; j < SAMPLE_POINTS; j++) {
      int n;
      dGeomID ray;
      dContactGeom contact[RAY_CONTACTS];
      
      ray = dCreateRay(raySpace, 1.01);
      dGeomRaySet(ray, ((double)i/(SAMPLE_POINTS-1)) * TERRAIN_SIZE,
		  ((double)j/(SAMPLE_POINTS-1))*TERRAIN_SIZE, 1.0, 0, 0, -1.0);
      
      n = dCollide(ray, (dGeomID)space, RAY_CONTACTS,
		   contact, sizeof(dContactGeom));
      double max_height = -1.0;
      for (int k = 0; k < n; k++) {
	if (contact[k].pos[2] > max_height) {
	  max_height = contact[k].pos[2];
	}
      }
      heights[i][j] = (1+dRandReal()*0.01) * max_height +
	dRandReal()*0.01 - 0.005;
      dGeomDestroy(ray);
    }
  }

  ofstream fout;

  fout.open("terrain.heightmap");
  fout << SAMPLE_POINTS << " " << SAMPLE_POINTS << " " 
       << TERRAIN_SIZE / ((double)(SAMPLE_POINTS - 1)) << " "
       << TERRAIN_SIZE / ((double)(SAMPLE_POINTS - 1)) << " 1.0" << endl;
    
  for (int i = 0; i < SAMPLE_POINTS; i++) {
    for (int j = 0; j < SAMPLE_POINTS; j++) {
      fout << heights[i][j] << " ";
    }
    fout << endl;
  }
  fout.close();

  Terrain terrain("terrain.heightmap", true);
  terrain.writeAsTrimesh("terrain.trimesh");
  
}

    


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions

  if (argc != 3) {
    printf("Usage: tergen <rand seed> <difficultly level>\n");
    exit(1);
  }
  
  // create world
  
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dWorldSetAutoDisableFlag (world,1);
  dWorldSetContactMaxCorrectingVel (world,0.1);
  dWorldSetContactSurfaceLayer (world,0.001);
  dCreatePlane (space,0,0,1,0);
  numBodies = 0;
  dRandSetSeed(atoi((char*)argv[1]));
  level = atoi((char*)argv[2]);

  if (level < 0 || level >= NUM_LEVELS) {
    printf("Level must be between 0 and %d.\n", NUM_LEVELS-1);
    exit(1);
  }

  for (int i = 0; i < (int)levelSettings[level][0]; i++) {
    dReal sides[3];
    dMass m;
    
    bodies[numBodies] = dBodyCreate(world);
    sides[0] = levelSettings[level][1] + dRandReal() *
      (levelSettings[level][2] - levelSettings[level][1]);
    sides[1] = levelSettings[level][3] + dRandReal() *
      (levelSettings[level][4] - levelSettings[level][3]);
    sides[2] = levelSettings[level][5] + dRandReal() *
      (levelSettings[level][6] - levelSettings[level][5]);

    geoms[numBodies] = dCreateBox (space,sides[0],sides[1],sides[2]);
    dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
    dBodySetPosition(bodies[numBodies], dRandReal()*TERRAIN_SIZE,
		     0.5 + dRandReal() * (TERRAIN_SIZE - 1.0), 5.0);
    dGeomSetBody (geoms[numBodies], bodies[numBodies]);
    dBodySetMass (bodies[numBodies], &m);
    numBodies++;

    for (int j = 0; j < 50; j++) {
      dSpaceCollide (space,0,&nearCallback);
      dWorldQuickStep (world,0.05);
      dJointGroupEmpty (contactgroup);
    }
  }
  sampleHeightmap();

  
  dSpaceDestroy (space);
  dWorldDestroy (world);
  return 0;
}
