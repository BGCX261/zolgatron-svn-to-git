#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDUtils.h"

#include <algorithm>
#include <cassert>
#include <ext/hash_set>
#include <iostream>
#include <queue>
#include <sstream>
#include <vector>


using namespace std;

struct VInfo {
  int i;
  int j;
  float x;
  float y;
  float h;

  float maxh;
  bool done;
};

struct VNode {
  VNode* prev;
  VInfo vinfo;
  float g;
  float h;
};


VInfo infos[101][101];

int goal_i = 2.75 / 3.0 * 100;
int goal_j = 0;

float dist_to_goal(float x, float y) {
  float dx = x - 2.75;
  float dy = y;
  float d = sqrt(dx*dx + dy*dy);
  return d;
}

float dist_2d(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  float d = sqrt(dx*dx + dy*dy);
  return d;
}

void compute_maxh(int i, int j) {
  bool start = true;
  float maxh = 0;
  float minh = 0;
  for (int ni = i - 4; ni <= i + 4; ++ni) {
    for (int nj = j - 4; nj <= j + 4; ++nj) {
      if (ni < 0 || nj < 0) continue;
      if (ni > 100 || nj > 100) continue;
      if (start) {
        start = false;
        maxh = infos[ni][nj].h;
        minh = infos[ni][nj].h;
      }
      if (infos[ni][nj].h > maxh) {
        maxh = infos[ni][nj].h;
      }
      if (infos[ni][nj].h < minh) {
        minh = infos[ni][nj].h;
      }
    }
  }
  infos[i][j].maxh = maxh - minh;
}

void compute_all_maxh() {
  for (int i = 0; i <= 100; ++i) {
    for (int j = 0; j <= 100; ++j) {
      compute_maxh(i, j);
    }
  }
}

void print_path(VNode* node) {
  if (node->prev) {
    print_path(node->prev);
    cout << " " << node->vinfo.i << ":" << node->vinfo.j << " ";
  }
}

void get_path(SDSimulator* sim, vector<bduVec3f>* path) {
  path->clear();

  for (int i = 0; i <= 100; ++i) {
    for (int j = 0; j <= 100; ++j) {
      float x = i / 100.0 * 3;
      float y = j / 100.0 * 3 - 1.5;
      float h = sim->getPointHeight(x, y);
      if (j == 50) {
        //cout << i << " " << h << endl;
      }
      VInfo info;
      info.i = i;
      info.j = j;
      info.x = x;
      info.y = y;
      info.h = h;
      info.maxh = 0;
      info.done = false;
      infos[i][j] = info;
    }
  }
  compute_all_maxh();

  //priority_queue<VNode*, vector<VNode*>, VNodeComparator<VNode*> > nodes;
  vector<VNode*> nodes;

  VNode* start = new VNode;
  start->vinfo = infos[0][50];
  start->prev = 0;
  start->g = 0;
  start->h = dist_to_goal(start->vinfo.x, start->vinfo.y);
  nodes.push_back(start);

  VNode* final = 0;
  //int nd = 0;

  while (!nodes.empty()) {
    int minindex = -1;
    float minval = 0;
    for (unsigned int i = 0; i < nodes.size(); ++i) {
      float f = nodes[i]->g + nodes[i]->h;
      if (minindex == -1 || f < minval) {
        minval = f;
        minindex = i;
      }
    }

    VNode* node = nodes[minindex];
    nodes.erase(nodes.begin() + minindex);
    VInfo info = node->vinfo;

    //cout << "d: " << dist_to_goal(info.x, info.y) << endl;
    if (dist_to_goal(info.x, info.y) < .01) {
      final = node;
      break;
    }

    for (int nx = info.i; nx <= info.i + 1; nx++) {
      for (int ny = info.j - 1; ny <= info.j + 1; ny++) {
        if (nx == info.i && ny == info.j) continue;
        if (nx < 0 || ny < 5) continue;
        if (nx > 100 || ny > 100 - 5) continue;

        VInfo& oinfo = infos[nx][ny];
        //if (oinfo.done) continue;
        //cout << nx << " - " << ny << endl;
        if (oinfo.maxh < .12) {
          VNode* n = new VNode;
          n->vinfo = oinfo;
          n->prev = node;
          n->g = node->g + dist_2d(info.x, info.y, oinfo.x, oinfo.y);
          n->h = dist_to_goal(oinfo.x, oinfo.y);
          for (unsigned int p = 0; p < nodes.size(); ++p) {
            if (nodes[p]->vinfo.i == nx && nodes[p]->vinfo.j == ny) {
              if (n->g > nodes[p]->g) {
                n->g = nodes[p]->g;
                n->prev = nodes[p]->prev;
              }
              nodes.erase(nodes.begin() + p);
              break;
            }
          }
          nodes.push_back(n);
        }
      }
    }
  }

  if (!final) {
    cout << "no solution" << endl;
    exit(1);
  }

  print_path(final);


  VNode* node = final;
  while (node->prev) {
    path->push_back(bduVec3f(node->vinfo.x, node->vinfo.y, 0));
    node = node->prev;
  }
  reverse(path->begin(), path->end());
  cout << endl << "path size is " << path->size() << endl;
}
