#ifndef __AMAR_ASTAR_H__
#define __AMAR_ASTAR_H__

#include <queue>
#include <utility>
#include <ext/hash_set>

#include "SDSlowWalk.h"

class SDLittleDog;
class SDSimulator;
class SDSlowWalk;

using namespace __gnu_cxx;

// define hash on strings to use in hash_set
namespace __gnu_cxx {
template<> struct hash< std::string > {
  size_t operator()( const std::string& x ) const
  {
    return hash< const char* >()( x.c_str() );
  }
};
}


namespace amar {


struct LittleDogNode {
  // Global position of each foot.
  bduVec3f foot[4];

  // Local position of each foot.
  bduVec3f localFoot[4];

  // No. of steps taken by the robot.
  double numSteps;

  // Heuristic no. of steps to goal.
  double numHeuristicSteps;

  // Foot and how much to move it
  int moveFoot;
  double x;
  double y;
  double z;

  // Path
  vector<SDDogStep> path;

  // State of the simulator.
  SDSimulatorState simulator_state;

  double cost() const {
    return numSteps;
  }

  double heuristic() const {
    return numHeuristicSteps;
  }

  const SDSimulatorState& const_simulator_state() const {
    return simulator_state;
  }
};

// Comparator for priority queue nodes in A* search.
//
// Class T should define cost() and heuristic() methods.
template<class T>
struct AStarComparator : public binary_function<T, T, bool> {
  bool operator() (const T& a, const T& b) const {
    // Priority of a is less than that of b if overall cost of a is
    // greater than that of b.
    return (a.cost() + a.heuristic() >= b.cost() + b.heuristic());
  }
};


// A step is valid if foot position is within 1 cm. of (x,y)
bool validStep(double x, double y, bduVec3f foot);
bduVec3f littleDogCenter(const bduVec3f* feet);

enum ClassifierMode {
  NONE = 0,
  TRAIN = 1,
  TEST = 2,
  USE = 3
};

class AStarStats {
 public:
  // No. of times tryMove was called
  int num_trials;

  // No. of steps which were actually simulated.
  int num_simulated;

  // Classifier said to try, and was a valid step.
  int true_positives;

  // Classifier said to try but was an invalid step.
  int false_positives;

  // Classifier said not to try, and was an invalid step.
  int true_negatives;

  // Classifier said not to try, but was a valid step.
  int false_negatives;

  AStarStats()
      : num_trials(0), num_simulated(0),
        true_positives(0), false_positives(0),
        true_negatives(0), false_negatives(0) {
  }
};

class AStar {
 public:
  AStar(ClassifierMode classifier_mode) {
    // TODO this is hardcoded for now
    fileStarted = false;
    highLevelMode = false;
    num_features_ = 23;
    done_training_ = false;

    classifier_mode_ = classifier_mode;

    for (int i = 0; i < 4; ++i) {
      Classifier* c = new Classifier(num_features_);
      classifiers.push_back(c);
      vector<Datum> v;
      datums.push_back(v);
    }

    if (classifier_mode_ == TEST || classifier_mode == USE) {
      cout << "Loading classifier weights" << endl;
      loadWeights();
    }
  }


  virtual ~AStar() {
    for (int i = 0; i < 4; ++i) {
      delete classifiers[i];
    }
  }


  void setGridSize(int size);
  void setSimulator(SDSimulator *simulator);
  void setLittleDog(SDLittleDog *littleDog);
  void setWalkController(SDSlowWalk *walkController);
  void setStepSize(double stepSize);
  void setGoal(double goal_x, double goal_y);
  void setNumFeatures(int num_features);
  void setTrainingSamples(int num_samples);

  bool tryMove(const LittleDogNode& node, SDSimulatorState *after);
  double distanceToGoal2d(const LittleDogNode& node);
  double distanceToGoal3d(const LittleDogNode& node);
  double heuristicSteps(const LittleDogNode& node);
  double heuristicSteps_1(const LittleDogNode& node);
  void search();
  
  void printStats() {
    cout << "\nStats:" << endl;
    cout << "num_trials     : " << stats_.num_trials << endl;
    cout << "num_simulated  : " << stats_.num_simulated << endl;
    cout << "true_positives : " << stats_.true_positives << endl;
    cout << "false_positives: " << stats_.false_positives << endl;
    cout << "true_negatives : " << stats_.true_negatives << endl;
    cout << "false_negatives: " << stats_.false_negatives << endl;
    cout << endl;
  }

  // Add a sample to classifier
  void addSample(const LittleDogNode& node, int foot,
                 const bduVec3f& localStep, bool is_valid);

  // Load weights from "classifier.txt" into per foot classifiers
  void loadWeights();

  // Save weights of per foot classifiers to "classifier.txt"
  void saveWeights();
  
  void writePath(const LittleDogNode& node) {
    ofstream fout;
    if(!fileStarted || !highLevelMode) {
      fout.open("actions.path");
      fileStarted = true;
    } else if(fileStarted && highLevelMode) {
      fout.open("actions.path",ios::app);   
    }
    for (unsigned int i = 0; i < node.path.size(); ++i) {
      fout << node.path[i].foot << " "
           << node.path[i].x << " "
           << node.path[i].y << endl;
    }
    fout.close();
  }

  void printPath(const LittleDogNode& node) {
    for (unsigned int i = 0; i < node.path.size(); ++i) {
      cout << node.path[i].foot << " "
           << node.path[i].x << " "
           << node.path[i].y << " -- ";
    }
    cout << endl;
  }

  bool highLevelMode;
  vector< pair<double,double> > anchor;

 private:
  bool fileStarted;
  int grid_size_;
  SDSimulator *simulator_;
  SDLittleDog *littleDog_;
  SDSlowWalk *walkController_;
  double stepSize_;
  bduVec3f goal_;
  bool use_classifier_;
  int num_features_;
  int num_samples_;
  int *foot_sequence_;
  

  // Set to true in addSample once all samples collected.
  bool done_training_;

  hash_set<string> history_;
  vector<bduVec3f *> history1_;

  ClassifierMode classifier_mode_;

  vector<Classifier*> classifiers;
  vector<vector<Datum> > datums;

  bool update_history(const bduVec3f* feet);
  bool update_history1(const bduVec3f* feet);
  bool validHeights_terrain(const bduVec3f* curr);
  bool validHeights(const bduVec3f* curr);
  AStarStats stats_;
};


}  // end namespace amar

#endif
