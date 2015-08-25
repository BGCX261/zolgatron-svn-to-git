#ifndef __CLASSIFIER__
#define __CLASSIFIER__

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

struct Datum {
  double* features;
  int num_features;
  int label;
};

class Classifier {
public:
  Classifier (int num_f) {
    num_features = num_f;
    weights = new double[num_features];
  }

  ~Classifier(){
    delete[] weights;
  }

  void printWeights(){
    printf("[");
    for (int i = 0; i < num_features; i++) {
      printf("%f ",weights[i]);
    }
    printf("]");
  }

  int getClassOf(Datum d);

  const double* getWeights() {
    return weights;
  }

  void setWeights(const double* w) {
    for (int i = 0; i < num_features; ++i) {
      weights[i] = w[i];
    }
  }

  void train(const vector<Datum>& data);

private:
  double* weights;
  int num_features;
};

#endif
