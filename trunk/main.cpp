#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDVectorOp.h"
#include "Classifier.h"
#include "aStar.h"
#include "amar_astar.h"
#include <iostream>
#include <fstream>
#include <drawstuff/drawstuff.h>
#include <string>
#include <vector>
#include <utility>

#define GOALX 2.75
#define GOALY 0.0
#define GRID_SIZE 5
#define MAXSTEP 0.05

using namespace std;

/**
 * Here is where all the magic happens.  With the
 * default setup, the -path flag will load "actions.path"
 * and attempt to simulate the footsteps line by line.
 * This is for us to see the robot walk along your path.
 * We have provided an example file to make the robot walk
 * .2 meters ahead.  Note that the simulation starts off
 * paused.  You can use Ctrl+P to move it ahead one step at
 * a time, or you can change the "pause" argument in
 * simulator/LittleDog.cpp to be something else.  To see
 * the terrain press 't' (note the terrain is simulated
 * even if it is not displayed graphically).
 *
 * Your job is to write the code that will generate
 * the path file - that is you must discretize the search
 * space, perform A* search using a logistic regression
 * classifier (trained before hand or on the fly), and
 * finally write the returned path to file.  Once the
 * milestone is submitted you may change virtually anything
 * you want to improve performance, including the controller
 * used.  When you submit, you will tell us the flags your
 * program needs to be run with in order to get the desired
 * behavior.
 */

bool runPath(string file, SDSimulator *simulator, SDLittleDog& littleDog,
	     SDSlowWalk& walkController){
  string line;
  ifstream paths;
  paths.open(file.c_str());
  if(!paths.is_open()){
    printf(("Error opening "+file+"\n").c_str());
    return false;
  }

  SDSimulatorState state;
  littleDog.setSimUI(true);
  //bduVec3f globalFeet[4];
  while(!paths.eof()){
    getline(paths,line);
    int index = line.find(" ");
    int foot = atoi((line.substr(0,index)).c_str());
    int index2 = line.find(" ",index+1);
    double x = atof((line.substr(index+1,index2-index-1)).c_str());
    double y = atof((line.substr(index2)).c_str());

    simulator->getState(&state);
    walkController.setStepPosition(foot, x, y);
    littleDog.runTrial(&state);
    simulator->getState(&state);
    // This prevents the simulator straying from search - see handout
    // faq (last page) for details
    simulator->setState(&state);
    //simulator->getGlobalFootPositions(globalFeet);
    //printf("1 %f %f\n", globalFeet[foot].n[0], globalFeet[foot].n[1]);
  }
  paths.close();
  return true;
}

bool runFullPath(string file, SDSimulator *simulator, SDLittleDog& littleDog,
	     SDSlowWalk& walkController){
	// Parse path steps.
	vector<SDDogStep> steps;
	parsePath(file, &steps);

  SDSimulatorState state;
	simulator->getState(&state);
  littleDog.setSimUI(true);
  littleDog.runPath(&state, &steps, &walkController);
  return true;
}

bool ifStepSuccess( double x, double y, bduVec3f globalFeet)
{
  double dist =
  sqrt((x-globalFeet.n[0])*(x-globalFeet.n[0])+((y-globalFeet.n[1])*(y-globalFeet.n[1])));
  if( dist<0.01)
    return true;
  else
    return false;
}

void genFeature( bduVec3f expect, bduVec3f *old, double * features)
{
  double dist;
  for(int i=0; i<4 ;i++){
    dist = 0;
    for(int j=0; j<3;j++){
      features[i*3+j] = old[i].n[j];
      if(i<3)
	dist +=  (old[i].n[j] - old[i+1].n[j])*(old[i].n[j] - old[i+1].n[j]);
    }
    if( i<3 )
      features[12+i] = sqrt(dist);
  }
  features[15] = expect.n[0];
  features[16] = expect.n[1];
  features[17] = expect.n[2];
 //  for(int i=0;i<18;i++){
//     printf("%.3f ", features[i]);
//   }
//   printf("\n");
}


void collectTrainingData(  vector<Datum>& data,  SDSimulator *simulator, SDLittleDog& littleDog,
			  SDSlowWalk& walkController, int num){
  SDSimulatorState state;
  bduVec3f globalFeet[4],  localFeet[4], expectedLocalFeet;
  double grid = MAXSTEP/5;
  double x, y, stepx, stepy;
  int pos =0, neg=0;
  littleDog.setSimUI(false);
  while( num > 0 ){
    for (int i=0; i<4; i++){
      Datum temp;
      temp.num_features = 18;

      //printf("Foot %d:\n", i);
      simulator->getGlobalFootPositions(globalFeet);
      simulator->getLocalFootPositions(localFeet);
      //printf("1 global %f %f %f\n", globalFeet[i].n[0],
      //		  globalFeet[i].n[1],  globalFeet[i].n[2]);

      //printf("1 local %f %f %f\n", localFeet[i].n[0],
      //	  localFeet[i].n[1],  localFeet[i].n[2]);
      stepx =  (rand()%5-2)*grid;
      stepy =  (rand()%5-2)*grid;
      x = globalFeet[i].n[0] + stepx;
      y = globalFeet[i].n[1] + stepy;
      simulator->getState(&state);
      walkController.setStepPosition(i, x, y);
      simulator->getLocalStepPosition(x, y, &expectedLocalFeet);
      genFeature(expectedLocalFeet, localFeet, temp.features );
      //printf("sss %f %f %f\n", x, y, simulator->getPointHeight(x,y));
      littleDog.runTrial(&state);
      simulator->getState(&state);
      simulator->getGlobalFootPositions(globalFeet);
      // printf("2 global %f %f %f\n", globalFeet[i].n[0],
      //     		  globalFeet[i].n[1],  globalFeet[i].n[2]);
      //printf("2 local %f %f %f\n", expectedLocalFeet.n[0],
      //	  expectedLocalFeet.n[1],  expectedLocalFeet.n[2]);
      if( ifStepSuccess( x, y, globalFeet[i]) ) {
	temp.label = 1;
	pos++;
      }else{
	temp.label = -1;
	neg++;
      }
      simulator->setState(&state);


      data.push_back(temp);
      //print out for debug//
      /* for (int i=0;i<data.size();i++){
	for(int j=0;j<18;j++){
	  printf("%.3f ", data[i].features[j]);
	}
	printf("\n");
	}*/

    }
    num--;
  }

  printf("Pos %d neg %d\n", pos, neg);

}


int main(int argc, char** argv){
  SDSimulator *simulator;
  SDLittleDog littleDog;
  SDSlowWalk walkController;
  SDTerrain* terrain;

  littleDog.setSimulated(true);
  littleDog.setController(&walkController);
  walkController.setLittleDog(&littleDog);
  littleDog.initializeSimulator();
  simulator = littleDog.getSimulator();

  if (argc > 1 && !strcmp(argv[1], "-path")) {
    //runPath("path.txt", simulator, littleDog, walkController);
    runFullPath("actions.path", simulator, littleDog, walkController);
    return 0;
  }

  amar::ClassifierMode cmode = amar::NONE;
  if (argc > 1) {
    string flag = argv[1];
    if (flag == "noclassifier") {
      cout << "Not using classifier" << endl;
      cmode = amar::NONE;
    } else if (flag == "train") {
      cout << "Training classifier" << endl;
      cmode = amar::TRAIN;
    } else if (flag == "test") {
      cout << "Testing classifier" << endl;
      cmode = amar::TEST;
    } else {
      cout << "Using classifier weights from classifier.txt" << endl;
      cmode = amar::NONE;
    }
  }

  littleDog.setSimUI(false);




  amar::AStar aStarSearch(cmode);
  aStarSearch.setTrainingSamples(500);

  aStarSearch.setGridSize(GRID_SIZE);
  aStarSearch.setStepSize(MAXSTEP);
  aStarSearch.setGoal(GOALX, GOALY);
  aStarSearch.setSimulator(simulator);
  aStarSearch.setLittleDog(&littleDog);
  aStarSearch.setWalkController(&walkController);
  
  //aStarSearch.highLevelMode = true;
  // try high level path
  terrain = simulator->getTerrain();
  vector< pair<double,double> > path;
  terrain->getPath(GOALX, GOALY+1.5, path);
 

  // break the high level path into sub-pathes
  int anchorStep = 5;
  int anchorNum = path.size()/anchorStep;
  if( path.size()%anchorStep > anchorStep/2 )
    anchorNum ++;
  for( int i=1; i<anchorNum; ++i){
    cout <<"temp goal "<< path[i*anchorStep].first<<", "<<path[i*anchorStep].second<<endl;
    //aStarSearch.setGoal(path[i*anchorStep].first,
    //		        path[i*anchorStep].second);
    //aStarSearch.search();
    aStarSearch.anchor.push_back(path[i*anchorStep]);
  }
  aStarSearch.anchor.push_back(make_pair(GOALX, GOALY));
  //aStarSearch.setGoal(GOALX, GOALY);
  
  aStarSearch.search();
  
  
  cout << aStarSearch.anchor.size()<< endl;
  for( int i=0; i<aStarSearch.anchor.size(); ++i){
    cout << aStarSearch.anchor[i].first <<"," << aStarSearch.anchor[i].second << endl;
  }

   // switches off visual display and does the search
  /*aStarSearch.search();
  if (cmode == amar::USE || cmode == amar::TEST) {
    aStarSearch.printStats();
    }
  */
  return 0;
}
