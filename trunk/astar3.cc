#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDUtils.h"
#include "SDVectorOp.h"
#include "Classifier.h"

#include <algorithm>
#include <cassert>
#include <ext/hash_set>
#include <iostream>
#include <queue>
#include <sstream>
#include <vector>

using namespace std;

float kStepSize = .05;   // single step size increment
float kMinStepSize = .01;
float goal_x = 2.75;
float goal_y = 0.0;

bduVec3f goal(goal_x, goal_y, 0);

void init(SDSimulator*& sim, SDLittleDog& dog, SDSlowWalk& control) {
  dog.setSimulated(true);
  dog.setController(&control);
  control.setLittleDog(&dog);
  dog.initializeSimulator();
  sim = dog.getSimulator();

  SDSimulatorState simState;
  sim->getState(&simState);
  sim->setState(&simState);
  dog.updateDogState();
}

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


enum OpType {
  OP_MAJOR = 0,
  OP_MINOR = 1,
  OP_NOOP  = 2
};

struct Op {
  int leg;
  float dx;
  float dy;
  OpType optype;
  Op() {
  }
  Op (int l, float x, float y, OpType ot) {
    leg = l;
    dx = x;
    dy = y;
    optype = ot;
  }
};


struct Node {
  bduVec3f gfoot[4];
  bduVec3f lfoot[4];
  SDSimulatorState sim_state;

  float g;  // cost
  float h;  // heuristic

  bool simulated;  // have we run simulator on this node
  Node* prev;      // prev node on which op was applied
  Op op;           // operation applied on the previous node

  bduVec3f op_pos;
  int vertex;

  bool willfall;
};


template<class T>
struct NodeComparator : public binary_function<T, T, bool> {
  bool operator() (const T& a, const T& b) const {
    // Priority of a is less than that of b if overall cost of a is
    // greater than that of b.
    float f_a = a->g + a->h;
    float f_b = b->g + b->h;
    return (f_a >= f_b);
  }
};


void make_all_ops(vector<Op>* ops) {
  ops->clear();
  for (int leg = 0; leg < 4; ++leg) {
    float dx = kStepSize;
    float dy = kStepSize * .6;
    ops->push_back(Op(leg, 0, -dy, OP_MAJOR));
    ops->push_back(Op(leg, 0, dy, OP_MAJOR));
    ops->push_back(Op(leg, dx, -dy, OP_MAJOR));
    ops->push_back(Op(leg, dx, 0, OP_MAJOR));
    ops->push_back(Op(leg, dx, -dy, OP_MAJOR));
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <=1; ++iy) {
        if (ix == 0 || iy == 0) continue;
        ops->push_back(Op(leg, ix*kMinStepSize, iy*kMinStepSize, OP_MINOR));
        ops->push_back(Op(leg, ix*kStepSize, iy*kStepSize, OP_MAJOR));
      }
    }
  }
}

void make_all_near_ops(vector<Op>* ops) {
  ops->clear();
  make_all_ops(ops);
  for (unsigned i = 0; i < ops->size(); ++i) {
    (*ops)[i].dx /= 5.0;
    (*ops)[i].dy /= 5.0;
  }
}


void add_bduvec3_4(bduVec3f f[], bduVec3f* v) {
  v->n[0] = (f[0].n[0] + f[1].n[0] + f[2].n[0] + f[3].n[0]);
  v->n[1] = (f[0].n[1] + f[1].n[1] + f[2].n[1] + f[3].n[1]);
  v->n[2] = (f[0].n[2] + f[1].n[2] + f[2].n[2] + f[3].n[2]);
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

string getHash(const bduVec3f* feet) {
  const int kg = 1 / kMinStepSize;
  stringstream s;
  s << int(feet[0].n[0] * kg) << ":" << int(feet[0].n[1] * kg) << ":"
    << int(feet[1].n[0] * kg) << ":" << int(feet[1].n[1] * kg) << ":"
    << int(feet[2].n[0] * kg) << ":" << int(feet[2].n[1] * kg) << ":"
    << int(feet[3].n[0] * kg) << ":" << int(feet[3].n[1] * kg);
  return s.str();
}


// If a state (represented by feet position) has already been
// visited, return true. If not, mark that state as visited and
// return false.
bool update_history(hash_set<string>* history, const bduVec3f* feet) {
  string hash = getHash(feet);
  bool present = (history->find(hash) != history->end());
  if (!present) {
    history->insert(hash);
    cout << ".";
  } else {
    cout << "*";
  }
  cout << flush;
  return present;
}

float distance_2d(float x1, float y1, float x2, float y2) {
  float dx = x1-x2;
  float dy = y1-y2;
  return sqrt(dx*dx + dy*dy);
}


float distance_to_goal_2d(bduVec3f v) {
  v.n[2] = 0;
  return SD_VEC_DIST(v, goal);
}

float distance_2d(bduVec3f v1, bduVec3f v2) {
  v1.n[2] = 0;
  v2.n[2] = 0;
  return SD_VEC_DIST(v1, v2);
}

float distance_2d(float x1, float y1, bduVec3f v) {
  return distance_2d(x1, y1, v.n[0], v.n[1]);
}

bduVec3f get_op_position(SDSimulator* sim, Op op) {
  bduVec3f gfoot[4];
  sim->getGlobalFootPositions(gfoot);

  bduVec3f pos, ori, footl, footg;
  bduVec4f qori;
  sim->getBodyInformation(LittleDog::B_TRUNK, &pos, &qori);
  SDQuaternionToEuler(qori, &ori);
  SDWorldToLocal(pos, ori, gfoot[op.leg], &footl);
  footl.n[0] += op.dx;
  footl.n[1] += op.dy;
  SDLocalToWorld(pos, ori, footl, &footg);
  footg.n[2] = sim->getPointHeight(footg.n[0], footg.n[1]);
  return footg;
}

float get_penalty_old(SDSimulator* sim, Op op) {
  bduVec3f gfoot[4];
  sim->getGlobalFootPositions(gfoot);

  bduVec3f center = littleDogCenter(gfoot);
  float dist1 = 0;
  for (int i = 0; i < 4; ++i) {
    dist1 += SD_VEC_DIST(center, gfoot[i]);
  }

  gfoot[op.leg] = get_op_position(sim, op);
  center = littleDogCenter(gfoot);
  float dist2 = 0;
  for (int i = 0; i < 4; ++i) {
    dist2 += SD_VEC_DIST(center, gfoot[i]);
  }
  return dist2 - dist1;
}


float get_penalty(SDSimulator* sim, Op op) {
  bduVec3f lfoot[4];
  sim->getLocalFootPositions(lfoot);
  lfoot[op.leg].n[0] += op.dx;
  lfoot[op.leg].n[1] += op.dy;

  float dy =
      fabs(lfoot[0].n[1]-.6) + fabs(lfoot[1].n[1]+.6) +
      fabs(lfoot[2].n[1]-.6) + fabs(lfoot[2].n[1]+.6);

  float penalty = dy - kStepSize;
  return penalty;
}


float compute_heuristic(SDSimulator* sim, Node* node,
                        vector<bduVec3f>& path, vector<float>& cdists) {
  bduVec3f s;
  assert(!node->simulated);
  bduVec3f gfoot[4];
  sim->getGlobalFootPositions(gfoot);
  gfoot[node->op.leg] = get_op_position(sim, node->op);
  s = littleDogCenter(gfoot);

  int v1 = node->prev->vertex;
  node->vertex = v1;

  float dist = distance_2d(path[v1], s) + cdists[v1];
  if (int(path.size()) > v1 + 1) {
    float dist_v = distance_2d(path[v1+1], s);
    dist = dist_v + cdists[v1+1];
    if (dist_v < .05) {
      node->vertex = v1 + 1;
    }
  }
  return dist/kStepSize * 4;
}


void bdu_vec3_print(bduVec3f v) {
  printf("%.3f %.3f %.3f\n", v.n[0], v.n[1], v.n[2]);
}

void write_path(Node* node, string fname) {
  cout << "writing to file " << fname << endl;
  vector<string> moves;
  char buffer[10000];
  while(node->prev != 0) {
    int leg = node->op.leg;
    float x = node->op_pos.n[0];
    float y = node->op_pos.n[1];
    sprintf(buffer, "%d %f %f", leg, x, y);
    moves.push_back(buffer);
    node = node->prev;
  }

  ofstream fout(fname.c_str());
  for (int i = moves.size() - 1; i >= 0; --i) {
    fout << moves[i] << endl;
  }
  fout.close();
}

void print_foot_pos(SDSimulator*& sim) {
  bduVec3f gfoot[4];
  bduVec3f lfoot[4];
  sim->getGlobalFootPositions(gfoot);
  sim->getLocalFootPositions(lfoot);
  cout << "Local foot" << endl;
  bdu_vec3_print(lfoot[0]);
  bdu_vec3_print(lfoot[1]);
  bdu_vec3_print(lfoot[2]);
  bdu_vec3_print(lfoot[3]);
  cout << "Global foot" << endl;
  bdu_vec3_print(gfoot[0]);
  bdu_vec3_print(gfoot[1]);
  bdu_vec3_print(gfoot[2]);
  bdu_vec3_print(gfoot[3]);

  bduVec3f body_pos, leg_pos[4], pos;
  bduVec4f qori;
  sim->getBodyInformation(LittleDog::B_TRUNK, &body_pos, &qori);
  sim->getBodyInformation(LittleDog::B_FL_LLEG, &leg_pos[0], &qori);
  sim->getBodyInformation(LittleDog::B_FR_LLEG, &leg_pos[1], &qori);
  sim->getBodyInformation(LittleDog::B_HL_LLEG, &leg_pos[2], &qori);
  sim->getBodyInformation(LittleDog::B_HR_LLEG, &leg_pos[3], &qori);
  cout << "Local legs" << endl;
  for (int i = 0; i < 4; ++i) {
    sim->getLocalStepPosition(leg_pos[i].n[0], leg_pos[i].n[1], &pos);
    bdu_vec3_print(pos);
  }
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

void print_path(Node* node) {
  if (node->prev == 0) return;
  print_path(node->prev);
  printf(" [%d: %.2f %.2f] ", node->op.leg, node->op.dx, node->op.dy);
}


void output_feature_vec3f(double* f, int index, bduVec3f v) {
  f[index] = v.n[0];
  f[index + 1] = v.n[1];
  f[index + 2] = v.n[2];
}

const int  kNumFeatures = 37;

void output_feature(SDSimulator* sim, double* f, Op op) {
  bduVec3f gfoot[4];
  bduVec3f lfoot[4];
  sim->getGlobalFootPositions(gfoot);
  sim->getLocalFootPositions(lfoot);

  bduVec3f newpos = get_op_position(sim, op);
  float x = newpos.n[0];
  float y = newpos.n[1];
  bduVec3f step;
  sim->getLocalStepPosition(x, y, &step);

  bduVec3f angles[4];
  sim->getDogAngles(angles);

  f[0] = 1;
  output_feature_vec3f(f, 1, lfoot[0]);
  output_feature_vec3f(f, 4, lfoot[1]);
  output_feature_vec3f(f, 7, lfoot[2]);
  output_feature_vec3f(f, 10, lfoot[3]);
  output_feature_vec3f(f, 13, step);

  f[16] = SD_VEC_DIST(lfoot[0], lfoot[1]);
  f[17] = SD_VEC_DIST(lfoot[1], lfoot[2]);
  f[18] = SD_VEC_DIST(lfoot[2], lfoot[3]);
  f[19] = SD_VEC_DIST(lfoot[3], lfoot[4]);

  f[20] = sqrt(step.n[0] * step.n[0] + step.n[1] * step.n[1]);

  output_feature_vec3f(f, 21, angles[0]);
  output_feature_vec3f(f, 24, angles[1]);
  output_feature_vec3f(f, 27, angles[2]);
  output_feature_vec3f(f, 30, angles[3]);

  f[33] = distance_2d(.8, .6, lfoot[0]);
  f[34] = distance_2d(.8, -.6, lfoot[1]);
  f[35] = distance_2d(-.8, .6, lfoot[2]);
  f[36] = distance_2d(-.8, -.6, lfoot[3]);

  assert(kNumFeatures == 37);
}

void write_feature(ofstream& fout, vector<float> f) {
  for (int i = 0; i < kNumFeatures; ++i) {
    fout << f[i] << " ";
  }
  fout << endl;
}


void save_weights(vector<Classifier*> classifiers) {
  ofstream fout("classifier-amar.txt");
  for (int foot = 0; foot < 4; ++foot) {
    const double* weights = classifiers[foot]->getWeights();
    fout << foot << " ";
    for (int i = 0; i < kNumFeatures; ++i) {
      fout << weights[i] << " ";
    }
    fout << endl;
  }

  fout.close();
}

void load_weights(vector<Classifier*> classifiers) {
  ifstream fin("classifier-amar.txt");

  int foot;
  double* weights = new double[kNumFeatures];

  for (int i = 0; i < 4; ++i) {
    fin >> foot;
    for (int f = 0; f < kNumFeatures; ++f) {
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

enum ClassifierMode {
  NONE = 0,
  TRAIN = 1,
  TEST = 2,
  USE = 3
};

bool has_bad_height(SDSimulator* sim, Op op) {
  bduVec3f gfoot[4];
  sim->getGlobalFootPositions(gfoot);
  gfoot[op.leg] = get_op_position(sim, op);
  gfoot[op.leg].n[2] = sim->getPointHeight(
      gfoot[op.leg].n[0], gfoot[op.leg].n[1]);

  float minz = gfoot[0].n[2];
  float maxz = gfoot[0].n[2];
  for (int i = 1; i < 4; ++i) {
    minz = min(minz, gfoot[i].n[2]);
    maxz = max(minz, gfoot[i].n[2]);
  }
  if (maxz - minz > .25) {
    cout << "z: " << minz << " " << maxz << endl;
    return true;
  } else {
  return false;
  }
}

bool invalid_step(bduVec3f* f1, bduVec3f* f2, int leg) {
  for (int i = 0; i < 4; ++i) {
    if (i == leg) continue;
    float z1 = f1[i].n[2];
    float z2 = f2[i].n[2];
    if (fabs(z1 - z2) > .01) {
      cout << "Something is rotten............ " << z1 << " " << z2 << endl;
      return true;
    }
  }
  return false;
}


float get_max_height(SDSimulator* sim) {
  bduVec3f pos, ori;
  bduVec4f qori;
  sim->getBodyInformation(LittleDog::B_TRUNK, &pos, &qori);
  SDQuaternionToEuler(qori, &ori);

  bduVec3f l, g;
  float maxdz = -5;
  for (float x = 0; x <= .5; x += .01) {
    for (float y = -.12; y <= .13; y += .12) {
      l.n[0] = x;
      l.n[1] = y;
      l.n[2] = pos.n[2];
      SDLocalToWorld(pos, ori, l, &g);
      float dz = sim->getPointHeight(g.n[0], g.n[1]) - g.n[2];
      maxdz = max(maxdz, dz);
    }
  }
  cout << "maxdz ====== " << maxdz << endl;
  return maxdz;
}


bool has_bad_orientation(SDSimulator* sim) {
  return get_max_height(sim) > 0;
}


float get_diff_from_best(bduVec3f* l) {
  // bduVec3f c = littleDogCenter(l);
  // cout << "center = ";
  // SD_VEC_PRINT(c);

  float lx =
      fabs(l[0].n[0] - .08) + fabs(l[1].n[0] - .08) +
      fabs(l[2].n[0] + .08) + fabs(l[3].n[0] + .08);
  float ly =
      fabs(l[0].n[1] - .06) + fabs(l[1].n[1] + .06) +
      fabs(l[2].n[1] - .06) + fabs(l[3].n[1] + .06);
  return lx + ly;
}

bool bad_op_close(bduVec3f* l, Op op) {
  int leg = op.leg;
  float x = l[leg].n[0];
  float y = l[leg].n[1];

  bool ret = false;

  if (leg == 0) {
    if (x < .04 && op.dx < 0) ret = true;
    if (y < .03 && op.dy < 0) ret = true;
  }

  if (leg == 1) {
    if (x < .04 && op.dx < 0) ret = true;
    if (y > -.03 && op.dy > 0) ret = true;
  }

  if (leg == 2) {
    if (x > -.04 && op.dx > 0) ret = true;
    if (y < .03 && op.dy < 0) ret = true;
  }

  if (leg == 3) {
    if (x > -.04 && op.dx > 0) ret = true;
    if (y > -.03 && op.dy > 0) ret = true;
  }

  if (ret) {
    cout << "bad_op_close: ldp= " << leg << " "
         << op.dx << " " << op.dy << " "
         << x << " " << y << endl;
  }
  return ret;
}


bool bad_op_far(bduVec3f* l, Op op) {
  int leg = op.leg;
  float x = l[leg].n[0];
  float y = l[leg].n[1];

  bool ret = false;

  if (leg == 0) {
    if (x > .15 && op.dx > 0) ret = true;
    if (y > .1 && op.dy > 0) ret = true;
  }

  if (leg == 1) {
    if (x > .15 && op.dx > 0) ret = true;
    if (y < -.1 && op.dy < 0) ret = true;
  }

  if (leg == 2) {
    if (x < -.15 && op.dx < 0) ret = true;
    if (y > .1 && op.dy > 0) ret = true;
  }

  if (leg == 3) {
    if (x < -.15 && op.dx < 0) ret = true;
    if (y < -.1 && op.dy < 0) ret = true;
  }

  if (ret) {
    cout << "bad_op_FAR: ldp= " << leg << " "
         << op.dx << " " << op.dy << " "
         << x << " " << y << endl;
  }
  return ret;
}

void get_path(SDSimulator* sim, vector<bduVec3f>* path);

int main(int argc, char** argv) {
  SDSimulator* sim;
  SDLittleDog dog;
  SDSlowWalk control;

  hash_set<string> history;

  init(sim, dog, control);
  dog.setSimUI(false);

  vector<Op> all_ops, all_near_ops;
  make_all_ops(&all_ops);
  make_all_near_ops(&all_near_ops);

  bduVec3f goal;
  SD_VEC_SET(goal, 2.75, 0, 0);

  priority_queue<Node*, vector<Node*>, NodeComparator<Node*> > nodes;

  vector<bduVec3f> path;
  get_path(sim, &path);

  vector<float> cdists;
  cout << "pathsize " << path.size() << endl;
  cdists.resize(path.size());
  bduVec3f last = goal;
  float cdist = 0;
  for (int i = int(path.size()) - 1 ; i >= 0; --i) {
    cdist += distance_2d(last, path[i]);
    cdists[i] = cdist;
    last = path[i];
  }

  Node* start = new Node();
  sim->getState(&start->sim_state);
  sim->setState(&start->sim_state);
  sim->getGlobalFootPositions(start->gfoot);
  sim->getLocalFootPositions(start->lfoot);
  start->simulated = true;
  start->prev = 0;
  start->g = 0;
  start->h = 0;
  start->op = Op(3, 0, 0, OP_MAJOR);
  start->vertex = 0;
  nodes.push(start);

  // initialize classifiers
  vector<Classifier*> classifiers;
  vector<vector<Datum> > datums;
  for (int i = 0; i < 4; ++i) {
    Classifier* c = new Classifier(kNumFeatures);
    classifiers.push_back(c);
    vector<Datum> v;
    datums.push_back(v);
  }

  int mode = USE;
  int num_popped = 0;
  int last_save = 0;

  load_weights(classifiers);

  while(!nodes.empty()) {
    Node* node = nodes.top();
    nodes.pop();
    num_popped++;
    cout << "pop h= " << node->h << " g= " << node->g
         << " v= " << node->vertex << endl;

    if (!node->simulated) {
      SDSimulatorState s = node->prev->sim_state;
      sim->setState(&s);

      //print_foot_pos(sim);
      printf("op: %d %.2f %.2f\n", node->op.leg, node->op.dx, node->op.dy);

      Datum d;
      if (mode == TRAIN) {
        d.features = new double[kNumFeatures];
        d.num_features = kNumFeatures;
        output_feature(sim, d.features, node->op);
      }


      int leg = node->op.leg;
      bduVec3f newpos = get_op_position(sim, node->op);

      bduVec3f gfoot[4];
      sim->getGlobalFootPositions(gfoot);
      gfoot[leg] = newpos;
      if (mode != TRAIN) {
        if (history.find(getHash(gfoot)) != history.end()) {
          cout << "in history" << endl;
          continue;
        }
      }

      node->op_pos = newpos;
      control.setStepPosition(leg, newpos.n[0], newpos.n[1]);
      dog.runTrial(&s);
      sim->getState(&node->sim_state);
      sim->setState(&node->sim_state);
      sim->getGlobalFootPositions(node->gfoot);
      sim->getLocalFootPositions(node->lfoot);
      node->simulated = true;
      history.insert(getHash(node->gfoot));

      bool fallen = has_fallen(sim);
      float d2d = distance_2d(node->gfoot[leg], newpos);
      float mindist = (node->op.optype == OP_MAJOR) ? .01 : .01;
      bool invalid = d2d > mindist;
      invalid = invalid || invalid_step(gfoot, node->gfoot, leg);

      if (mode == TRAIN) {
        d.label = fallen || invalid ? 1 : 0;
        datums[node->op.leg].push_back(d);
        if (datums[node->op.leg].size() % 50 == 0) {
          cout << "Train samples for foot " << node->op.leg << ": "
               << datums[node->op.leg].size() << endl;
        }

        if (datums[node->op.leg].size() == 300) {
          cout << "Training classifiers..." << endl;
          for (int i = 0; i < 4; ++i) {
            cout << "Foot " << i << ":" << endl;
            classifiers[i]->train(datums[i]);
          }
          cout << "Saving weights..." << endl;
          save_weights(classifiers);
          return 0;
        }
      }

      if (invalid) {
        cout << "invalid step " << d2d << endl;
        continue;
      }

    }

    sim->setState(&node->sim_state);
    //print_path(node);
    cout << endl;
    if (has_fallen(sim)) {
      printf("fallen\n\n\n");
      continue;
    }
    printf("\n\n\n");

    if (num_popped > last_save + 100) {
      write_path(node, "actions.path");
      last_save = num_popped;
    }

    bduVec3f center = littleDogCenter(node->gfoot);
    float dist = SD_VEC_DIST(center, goal);
    if (dist < .02) {
      cout << "Mostly done..." << endl;
      write_path(node, "actions.path");
      return 0;
    }

    for (unsigned i = 0; i < all_ops.size(); ++i) {
      Op op = all_ops[i];
      Node* n = new Node();

      int oldleg = node->op.leg;
      int leg = op.leg;
      if (op.optype == OP_MAJOR) {
        if (oldleg == 0 && leg != 1) continue;
        if (oldleg == 1 && leg != 2) continue;
        if (oldleg == 2 && leg != 3) continue;
        if (oldleg == 3 && leg != 0) continue;
      }

      bool bad_op = bad_op_close(node->lfoot, op);
      if (bad_op) continue;

      bad_op = bad_op_far(node->lfoot, op);
      if (bad_op) continue;

      if (mode == USE) {
        Datum d;
        d.features = new double[kNumFeatures];
        d.num_features = kNumFeatures;
        output_feature(sim, d.features, op);
        bool willfall = classifiers[op.leg]->getClassOf(d);
        n->willfall = willfall;
        if (willfall) {
          //printf("willfall %d %.2f %.2f\n", op.leg, op.dx, op.dy);
          continue;
        }
      }

      n->simulated = false;
      n->prev = node;
      n->op = op;
      n->g = node->g + 1;
      n->h = compute_heuristic(sim, n, path, cdists);
      nodes.push(n);
      //printf("push %d %.2f %.2f %.2f %.2f\n", op.leg,op.dx,op.dy,n->g,n->h);
    }
  }
}
