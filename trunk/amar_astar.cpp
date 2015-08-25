#include <set>
#include <sstream>
#include <fstream>
#include <cassert>

#include "Classifier.h"
#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDVectorOp.h"
#include "SDUtils.h"
#include "amar_astar.h"
#include <algorithm>
#include <queue>

namespace amar {


void copyFeet(const bduVec3f* from, bduVec3f* to);

void fillFeatures(const bduVec3f* localFoot,
                  const bduVec3f& localStep,
                  double* features,
                  int foot);

// Num of features per datum in classifier.
int kNumFeatures = 23;

// Check all the foot to be within 1cm of previous/commanded locations
bool validStep3d(bduVec3f* oldFeet, bduVec3f* newFeet) {
  for(int i=0; i<4; ++i) {
    double dist = SDDistance(oldFeet[i], newFeet[i], 3);

    if(dist > 0.015) {
      cout << "$  " << flush;
      return false;
    }
  }
  return true;
}

bool has_fallen(SDSimulator* sim) {
  bduVec3f body_pos, leg_pos[4];
  bduVec4f qori;
  sim->getBodyInformation(LittleDog::B_TRUNK, &body_pos, &qori);
  sim->getBodyInformation(LittleDog::B_FL_LLEG, &leg_pos[0], &qori);
  sim->getBodyInformation(LittleDog::B_FR_LLEG, &leg_pos[1], &qori);
  sim->getBodyInformation(LittleDog::B_HL_LLEG, &leg_pos[2], &qori);
  sim->getBodyInformation(LittleDog::B_HR_LLEG, &leg_pos[3], &qori);
  float dz0 = leg_pos[0].n[2] - body_pos.n[2];
  float dz1 = leg_pos[1].n[2] - body_pos.n[2];
  float dz2 = leg_pos[2].n[2] - body_pos.n[2];
  float dz3 = leg_pos[3].n[2] - body_pos.n[2];
  //printf("dz: %.2f %.2f %.2f %.2f\n", dz0, dz1, dz2, dz3);
  return dz0 > 0 && dz1 > 0 && dz2 > 0 && dz3 > 0;
}

// Returns true if prevFoot heights - new Foot heights are within
// approximately the same (otherwise it has fallen over
bool AStar::validHeights(const bduVec3f* curr) {
  double dz;
  double dzTotal = 0;
  for(int leg=0; leg<4; ++leg) {
    double z = simulator_->getPointHeight(curr[leg].n[0], curr[leg].n[1]);
    dz = z - curr[leg].n[2];
    dz = dz > 0 ? dz : -dz;
    dzTotal += dz;
    if(dz > 0.015 || dzTotal > 0.1) { 
      cout << "CAUGHT IT!" << endl << flush;
      return false;
    }
  }
  // all heights are approx the same as prev
  return true;
}


// Returns true if centroid is within four feet
// simple balance check
bool balanceCheck(const bduVec3f* local) {
  for(int i=0; i<4; ++i)
    cout << local[i].n[0] << local[i].n[1] << endl;
  if(local[0].n[0]<0.01 || local[0].n[1]<0.01)
    return false;
  if(local[1].n[0]<0.01 || local[1].n[1]>-0.01)
    return false;
  if(local[2].n[0]>-0.01 || local[2].n[1]<0.01)
    return false;
  if(local[3].n[0]>-0.01 || local[3].n[1]>-0.01)
    return false;
  return true;
}


string getHash(const bduVec3f* feet) {
  const int kg = 100;
  stringstream s;
  s << int(feet[0].n[0] * kg) << ":" << int(feet[0].n[1] * kg) << ":"
    << int(feet[1].n[0] * kg) << ":" << int(feet[1].n[1] * kg) << ":"
    << int(feet[2].n[0] * kg) << ":" << int(feet[2].n[1] * kg) << ":"
    << int(feet[3].n[0] * kg) << ":" << int(feet[3].n[1] * kg);
  return s.str();
}


/*
 * This is stored in list of explored nodes, later used to avoid
 * exploring same nodes again.
 */
string getHash1(const bduVec3f* feet) {
  const int kg = 100;
  bduVec3f center = littleDogCenter(feet);
  vector<float> v;
  v.push_back(SD_VEC_DIST(center, feet[0]));
  v.push_back(SD_VEC_DIST(center, feet[1]));
  v.push_back(SD_VEC_DIST(center, feet[2]));
  v.push_back(SD_VEC_DIST(center, feet[3]));
  
  // TODO Check if sorting needs to be done, commenting for now.
  sort(v.begin(), v.end());

  stringstream s;
  // TODO Try to revert to log
  s << int(center.n[0] * kg) << ":" << int(center.n[1] * kg) << ":"
    << int(v[0]*100) << ":" << int(v[3]*100);
  return s.str();
}


bool AStar::tryMove(const LittleDogNode& node,
                    SDSimulatorState* after) {
  int foot = node.moveFoot;

  simulator_->setState(&node.const_simulator_state());
  littleDog_->updateDogState();
  //simulator_->getState(after);
  
  bduVec3f globalFeetOld[4];
  copyFeet(node.foot, globalFeetOld);

  globalFeetOld[foot].n[0] = node.x;
  globalFeetOld[foot].n[1] = node.y;
  globalFeetOld[foot].n[2] = node.z;
  
  double x = globalFeetOld[foot].n[0];
  double y = globalFeetOld[foot].n[1];
  
  stats_.num_trials++;

  // Record local step and set controller to move foot to new position.
  bduVec3f localStep;
  simulator_->getLocalStepPosition(x, y, &localStep);

  // Check the classifier results before simulating
  bool exploreNode = true;

  // If this is not training phase - call the classifier
  if (classifier_mode_ == USE || classifier_mode_ == TEST) {
    Datum d;
    d.features = new double[num_features_];
    d.num_features = num_features_;
    fillFeatures(node.localFoot, localStep, d.features, foot);
    exploreNode = classifiers[foot]->getClassOf(d);
  }

  // If classifier doesn't prune the node - or when no classifier is called
  bool isValid = false;

  if (exploreNode || classifier_mode_ == TEST) {
    // Simulate step.
    walkController_->setStepPosition(foot, x, y);
    littleDog_->runTrial(after);
    simulator_->getState(after);
    simulator_->setState(after);

    stats_.num_simulated++;

    // Get new foot positions.
    bduVec3f globalFeet[4];
    bduVec3f localFeet[4];
    simulator_->getGlobalFootPositions(globalFeet);
    simulator_->getLocalFootPositions(localFeet);

    // Check if trial successful.
    isValid =  validHeights(globalFeet)
               && validStep3d(globalFeetOld, globalFeet)
               && !has_fallen(simulator_);
               //&&balanceCheck(localFeet); //try different height check validHeights(node.foot, globalFeet, foot) 

    if (exploreNode && isValid) stats_.true_positives++;
    if (exploreNode && !isValid) stats_.false_positives++;
    if (!exploreNode && isValid) stats_.false_negatives++;
    if (!exploreNode && !isValid) stats_.true_negatives++;
  }

  // if this is training phase, add the sample
  if (classifier_mode_ == TRAIN) {
    addSample(node, foot, localStep, isValid);
  }

  return isValid;
}

void AStar::addSample(const LittleDogNode& node, int foot,
                      const bduVec3f& localStep, bool is_valid) {
  Datum d;
  d.features = new double[num_features_];
  d.num_features = num_features_;
  d.label = is_valid ? 1 : 0;

  fillFeatures(node.localFoot, localStep, d.features, foot);

  datums[foot].push_back(d);
  if (datums[foot].size() % 50 == 0) {
    cout << "Train samples for foot " << foot << ": "
         << datums[foot].size() << endl;
  }

  if (datums[foot].size() == num_samples_) {
    cout << "Training classifiers..." << endl;
    for (int i = 0; i < 4; ++i) {
      cout << "Foot " << i << ":" << endl;
      classifiers[i]->train(datums[i]);
    }
    cout << "Saving weights..." << endl;
    saveWeights();
    done_training_ = true;
  }
}


bduVec3f littleDogCenter(const bduVec3f* feet) {
  bduVec3f center;

  SD_VEC_ZERO(center);
  SD_VEC_ADD(center, feet[0], feet[1]);
  SD_VEC_ADD(center, center, feet[2]);
  SD_VEC_ADD(center, center, feet[3]);
  center.n[0] /= 4;
  center.n[1] /= 4;
  center.n[2] /= 4;

  return center;
}


bool AStar::update_history(const bduVec3f* feet) {
  const double dist = 0.01;

  for(int i=0;i<history1_.size();++i) {
    double dist0 = SD_VEC_DIST(history1_[i][0], feet[0]);
    double dist1 = SD_VEC_DIST(history1_[i][1], feet[1]);
    double dist2 = SD_VEC_DIST(history1_[i][2], feet[2]);
    double dist3 = SD_VEC_DIST(history1_[i][3], feet[3]);
    //cout << dist0 << "\t" << dist1 << "\t" << dist2 << "\t" << dist3 << endl;
    if(dist0 <= dist &&
       dist1 <= dist &&
       dist2 <= dist &&
       dist3 <= dist)
    {
      cout << "*";
      return true; 
    }
  }
  bduVec3f* tempFeet = new bduVec3f[4];
  copyFeet(feet, tempFeet);
  history1_.push_back(tempFeet);
  cout << ".";

  return false;
}

// If a state (represented by feet position) has already been
// visited, return true. If not, mark that state as visited and
// return false.
bool AStar::update_history1(const bduVec3f* feet) {
  string hash = getHash(feet);
  bool present = (history_.find(hash) != history_.end());
  if (!present) {
    history_.insert(hash);
    cout << ".";
  } else {
    cout << "*";
  }
  cout << flush;
  return present;
}


void AStar::search() {
  // Priority queue for the search.
  typedef LittleDogNode Node;
  priority_queue<Node, vector<Node>, AStarComparator<Node> > queue;

  // Get starting feet position.
  Node first;

  littleDog_->updateDogState();
  simulator_->getState(&first.simulator_state);
  simulator_->setState(&first.simulator_state);

  simulator_->getGlobalFootPositions(first.foot);
  simulator_->getLocalFootPositions(first.localFoot);

  cout << "Starting position of robot: ";
  SD_VEC_PRINT(littleDogCenter(first.foot));

  // Initialize priority queue with initial position of robot.
  first.numSteps = 0;
  first.numHeuristicSteps = heuristicSteps_1(first);


  first.moveFoot = 2;
  first.x = first.foot[first.moveFoot].n[0];
  first.y = first.foot[first.moveFoot].n[1];
  first.z = first.foot[first.moveFoot].n[2];
  queue.push(first);

  Node second(first);
  second.moveFoot = 1;
  second.x = second.foot[second.moveFoot].n[0];
  second.y = second.foot[second.moveFoot].n[1];
  second.z = second.foot[second.moveFoot].n[2];
  queue.push(second);

  Node third(first);
  third.moveFoot = 3;
  third.x = third.foot[third.moveFoot].n[0];
  third.y = third.foot[third.moveFoot].n[1];
  third.z = third.foot[third.moveFoot].n[2];
  queue.push(third);

  Node fourth(first);
  fourth.moveFoot = 0;
  fourth.x = fourth.foot[fourth.moveFoot].n[0];
  fourth.y = fourth.foot[fourth.moveFoot].n[1];
  fourth.z = fourth.foot[fourth.moveFoot].n[2];
  queue.push(fourth);
  

  //update_history(first.foot);
  bool isFirstNode = true;
  int popCount = 1;
  while (!queue.empty()) {
    popCount++;
    // Get highest priority node to process.
    Node node = queue.top();
    queue.pop();
    
    //printStats();

    cout << "QUEUE: " << queue.size() << endl;

    // center within some range of goal
    double goal_distance = distanceToGoal2d(node);
    if (!highLevelMode && popCount % 25 == 0) {
      cout << "WRITING PATH......" << endl;
      writePath(node);
    }

    if (goal_distance <= 0.10) {
      writePath(node);
      history1_.clear(); // for later subpath search
      break;
    }

    cout << "path size = " << node.path.size() << endl;
    //if (node.path.size() == 30) {
    //  writePath(node);
    // return;
    //}

    bool showPath = false;
    if (showPath && node.path.size() > 0) {
      cout << "showing path of size " << node.path.size() << endl;
      littleDog_->setSimUI(true);
      littleDog_->runPath(&node.simulator_state, &node.path, walkController_);
      littleDog_->setSimUI(false);
    }


    SDSimulatorState after(node.const_simulator_state());

    bool isvalid = tryMove(node, &after);
   
    if (classifier_mode_ == TRAIN && done_training_) {
      // Training classifier is done, so return
      return;
    }

    if (isvalid || isFirstNode) {
      isFirstNode = false;
      bduVec3f newGlobalFeet[4];
      bduVec3f newLocalFeet[4];
      simulator_->getGlobalFootPositions(newGlobalFeet);
      simulator_->getLocalFootPositions(newLocalFeet);
    
      if (update_history(newGlobalFeet) == true) {
        // If this node has already been explored before, just pop it off
        // and move to next node
        continue;
      }
      
      cout << "Goal distance   : " << goal_distance << endl;
      cout << "Heuristic value : " << node.numHeuristicSteps  << endl;


      int foot = 0;
      switch(node.moveFoot) {
      case 0:
        foot = 1;
        break;
      case 1:
        foot = 3;
        break;
      case 2:
        foot = 0;
        break;
      case 3:
        foot = 2;
        break;
      }
      //for (int foot = 0; foot < 4; ++foot) {
        // Iterate through all grid positions.
        for (int dx = -grid_size_ / 2; dx <= grid_size_ / 2; ++dx) {
          for (int dy = -grid_size_ / 2; dy <= grid_size_ / 2; ++dy) {
            if (dx == 0 && dy == 0)
              continue;
            //if (foot == node.moveFoot)
            //  continue;

            // Move succeeded, add corresponding node to priority queue.
            Node n;

            SDSimulatorState copyState(after);
            n.simulator_state = copyState;

            n.path = node.path;

            // TODO Probable error point - should we be storing foot positions
            // that we wanted OR the foot positions that we actually got
            // after simulation ?
            // Method 1 - pos we commanded to move to is stored -
            // TODO is the LAST step stored this way ?
            SDDogStep step(node.moveFoot, node.x, node.y);
            // Method 2 - pos After Simulation is stored -
            // SDDogStep step(foot, globalFeet[foot].n[0], globalFeet[foot].n[1]);

            n.path.push_back(step);

            // all of previous path + current move
            copyFeet(newGlobalFeet, n.foot);
            copyFeet(newLocalFeet, n.localFoot);

            n.moveFoot = foot;
            n.x = n.foot[foot].n[0] + dx * stepSize_;
            n.y = n.foot[foot].n[1] + dy * stepSize_;
            n.z = simulator_->getPointHeight(n.x, n.y);
            
            n.numSteps = node.numSteps + 1 + n.z;

            // Calculates the approx heuristic value [without simulating the step]
            Node temp(n);
            temp.foot[foot].n[0] = temp.x;
            temp.foot[foot].n[1] = temp.y;
            temp.foot[foot].n[2] = temp.z;
            n.numHeuristicSteps = heuristicSteps_1(temp);
            
            bduVec3f center = littleDogCenter(temp.foot);

            double maxHtDiff = 0;
            for(int l = 0; l<4; ++l) {
              double dh = fabs(n.foot[foot].n[2] - node.foot[l].n[2]);
              maxHtDiff = maxHtDiff < dh ? dh : maxHtDiff;
            }
            
            //n.numHeuristicSteps += 5*maxHtDiff;
            // cout << "dist: " << distanceToGoal(n) << "\t";
            queue.push(n);
          }
        }
      //}
    }
  }
}


double AStar::distanceToGoal2d(const LittleDogNode& node) {
  bduVec3f center = littleDogCenter(node.foot);
  return SDDistance(center, goal_, 2);
}

void AStar::loadWeights() {
  ifstream fin("classifier.txt");

  int foot;
  double* weights = new double[num_features_];

  for (int i = 0; i < 4; ++i) {
    fin >> foot;
    for (int f = 0; f < num_features_; ++f) {
      fin >> weights[f];
    }
    classifiers[foot]->setWeights(weights);
    cout << "foot: ";
    classifiers[foot]->printWeights();
    cout << endl;
  }

  fin.close();
  delete[] weights;
}

void AStar::saveWeights() {
  ofstream fout("classifier.txt");

  for (int foot = 0; foot < 4; ++foot) {
    const double* weights = classifiers[foot]->getWeights();

    fout << foot << " ";
    for (int i = 0; i < num_features_; ++i) {
      fout << weights[i] << " ";
    }
    fout << endl;
  }

  fout.close();
}

double AStar::heuristicSteps(const LittleDogNode& node) {
  double dist = distanceToGoal2d(node);
  double heuristic_steps = 10 * 4 * dist / ((int(grid_size_/2) * stepSize_));
  return heuristic_steps;
}

double AStar::heuristicSteps_1(const LittleDogNode& node) {
  double min = 100;
  int vertice = 0;
  bduVec3f tmp;
  bduVec3f center = littleDogCenter(node.foot);
  for( int i=0; i<anchor.size(); i++){
    tmp.n[0] = anchor[i].first;
    tmp.n[1] = anchor[i].second;
    tmp.n[2] = simulator_->getPointHeight(tmp.n[0], tmp.n[1]);
      if( SD_VEC_DIST(center, tmp) < min ){
	min =  SD_VEC_DIST(center, tmp);
	vertice = i;
      }
  }
  tmp.n[0] = anchor[vertice].first;
  tmp.n[1] = anchor[vertice].second;
  tmp.n[2] = simulator_->getPointHeight(tmp.n[0], tmp.n[1]);
  //cout <<  "path "<<SD_VEC_DIST(center, tmp)<< endl;
  double dist =  SD_VEC_DIST(center, tmp)/2;
  dist += SDDistance(center, goal_, 2);
  double heuristic_steps = 10 * 4 * dist / ((int(grid_size_/2) * stepSize_));
  return heuristic_steps;
}

void AStar::setGridSize(int size) {
  grid_size_ = size;
}

void AStar::setSimulator(SDSimulator *simulator) {
  simulator_ = simulator;
}

void AStar::setLittleDog(SDLittleDog *littleDog) {
  littleDog_ = littleDog;
}

void AStar::setStepSize(double stepSize) {
  stepSize_ = stepSize;
}

void AStar::setWalkController(SDSlowWalk *walkController) {
  walkController_ = walkController;
}

void AStar::setGoal(double goal_x, double goal_y) {
  goal_.n[0] = goal_x;
  goal_.n[1] = goal_y;
  goal_.n[2] = 0;
}

void AStar::setNumFeatures(int num_features) {
  num_features_ = num_features;
}

void AStar::setTrainingSamples(int num_samples) {
  num_samples_ = num_samples;
}


// Returns true if foot is within 1 cm of (x, y)
bool validStep2d(double x, double y, bduVec3f foot) {
  // cout << "valid: " << x << " " << y << " "
  //      << foot.n[0] << " " << foot.n[1] << endl;
  // TODO add Z dimension
  double dx = foot.n[0] - x;
  double dy = foot.n[1] - y;
  double dist_squared = dx*dx + dy*dy;
  const double max_dist = .01;  // 1 cm.
  return (dist_squared <= max_dist * max_dist);
}

void copyFeet(const bduVec3f* from, bduVec3f* to) {
  SD_VEC_COPY(to[0], from[0]);
  SD_VEC_COPY(to[1], from[1]);
  SD_VEC_COPY(to[2], from[2]);
  SD_VEC_COPY(to[3], from[3]);
}

void fillFeatures(const bduVec3f* localFoot,
                  const bduVec3f& localStep,
                  double* features,
                  int foot) {
  // First X = 1 - constant term
  features[0]  = 1;

  features[1]  = localFoot[0].n[0];
  features[2]  = localFoot[0].n[1];
  features[3]  = localFoot[0].n[2];

  features[4]  = localFoot[1].n[0];
  features[5]  = localFoot[1].n[1];
  features[6]  = localFoot[1].n[2];

  features[7]  = localFoot[2].n[0];
  features[8]  = localFoot[2].n[1];
  features[9]  = localFoot[2].n[2];

  features[10] = localFoot[3].n[0];
  features[11] = localFoot[3].n[1];
  features[12] = localFoot[3].n[2];

  features[13] = localStep.n[0];
  features[14] = localStep.n[1];
  features[15] = localStep.n[2];

  // Distances between the legs BEFORE the move is made
  features[16] = SD_VEC_DIST(localFoot[0], localFoot[1]);
  features[17] = SD_VEC_DIST(localFoot[1], localFoot[2]);
  features[18] = SD_VEC_DIST(localFoot[2], localFoot[3]);

  // Distances between the legs AFTER the move is made
  bduVec3f tempLocalFoot[4];
  copyFeet(localFoot, tempLocalFoot);
  SD_VEC_COPY(tempLocalFoot[foot], localStep);

  features[19] = SD_VEC_DIST(tempLocalFoot[0], tempLocalFoot[1]);
  features[20] = SD_VEC_DIST(tempLocalFoot[1], tempLocalFoot[2]);
  features[21] = SD_VEC_DIST(tempLocalFoot[2], tempLocalFoot[3]);

  // Distance between the BEFORE and AFTER position of the moved leg
  features[22] = SD_VEC_DIST(localFoot[foot], localStep);

  // Verfiy if num features set correctly
  assert(kNumFeatures == 23);
}

}  // end namespace amar
