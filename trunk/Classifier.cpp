#include "Classifier.h"
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <sstream>

using namespace std;

double dot(const double* a, const double* b, int dim) {
  double r = 0;
  for (int i = 0; i < dim; ++i) {
    r += a[i] * b[i];
  }
  return r;
}

int Classifier::getClassOf(Datum d) {
  double score =  dot(d.features, weights, num_features);
  return (score > .5);
}

void Classifier::train(const vector<Datum>& data){
  int m = data.size();   // Number of samples.
  int n = num_features;  // Number of features in each sample.

  float alpha = .1;
  for (int i = 0; i < n; ++i) {
    weights[i] = .1;
  }

  for (int iter = 0; iter < 1000; ++iter) {
    for (int i = 0; i < m; ++i) {
      const Datum& d = data[i];
      double s = dot(d.features, weights, n);
      double h = 1.0 / (1 + exp(-s));

      for (int j = 0; j < n; ++j) {
        weights[j] += alpha * (d.label - h) * d.features[j];
      }
    }

    cout << "[ ";
    for (int j = 0; j < n; ++j) {
      cout << weights[j] << " ";
    }
    cout << " ]" << endl;
  }
}
