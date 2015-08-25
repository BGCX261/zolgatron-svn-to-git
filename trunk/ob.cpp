#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDVectorOp.h"

#include <iostream>

using namespace std;

SDSimulator* sim;
SDLittleDog dog;
SDSlowWalk control;
bduVec3f gfeet[4], lfeet[4];


void init() {
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


void print_foot_pos() {
  sim->getGlobalFootPositions(gfeet);
  sim->getLocalFootPositions(lfeet);
  cout << "Local foot" << endl;
  SD_VEC_PRINT(lfeet[0]);
  SD_VEC_PRINT(lfeet[1]);
  SD_VEC_PRINT(lfeet[2]);
  SD_VEC_PRINT(lfeet[3]);
  cout << "Global foot" << endl;
  SD_VEC_PRINT(gfeet[0]);
  SD_VEC_PRINT(gfeet[1]);
  SD_VEC_PRINT(gfeet[2]);
  SD_VEC_PRINT(gfeet[3]);
}


void test_one_leg_move() {
  SDSimulatorState simState;
  sim->getState(&simState);
  sim->setState(&simState);

  for (int i = 0; i < 20; ++i) {
    SDSimulatorState state = simState;
    sim->getState(&state);
    sim->setState(&state);
    control.setStepPosition(0, i * .03, .1);
    dog.runTrial(&simState);
    sim->getGlobalFootPositions(gfeet);
    sim->getLocalFootPositions(lfeet);
    cout << i*.03 << " : ";
    SD_VEC_PRINT(gfeet[0]);
    sim->getState(&state);
    sim->setState(&state);
  }
}


void test_all_legs_move() {
  SDSimulatorState simState;
  sim->getState(&simState);
  sim->setState(&simState);

  for (int i = 0; i < 20; ++i) {
    int leg = i % 4;
    float dist = i / 4;
    sim->getGlobalFootPositions(gfeet);
    float leg_x = gfeet[leg].n[0];
    float leg_y = gfeet[leg].n[1];
    control.setStepPosition(leg, leg_x + .05, leg_y);
    dog.runTrial(&simState);
    sim->getState(&simState);
    sim->setState(&simState);
    sim->getGlobalFootPositions(gfeet);
    sim->getLocalFootPositions(lfeet);
    cout << leg << " " << leg_x + .05 << " : ";
    SD_VEC_PRINT(gfeet[leg]);
  }
}


void test_cross_leg_fall() {
  SDSimulatorState simState;
  sim->getState(&simState);
  sim->setState(&simState);

  dog.setSimUI(true);
  print_foot_pos();

  bduVec4f qOri;
  bduVec3f pos;

  SDDogState dogState;
  dog.getDogState(&dogState);
  cout << "pos ";
  SD_VEC_PRINT(dogState.pos);
  sim->getBodyInformation(LittleDog::B_FL_LLEG, &pos, &qOri);
  cout << "leg pos ";
  SD_VEC_PRINT(pos);
  cout << endl << endl;

  control.setStepPosition(0, -.1, -.1);
  dog.runTrial(&simState);
  sim->getState(&simState);
  sim->setState(&simState);

  control.setStepPosition(3, .1, .1);
  dog.runTrial(&simState);
  sim->getState(&simState);
  sim->setState(&simState);

  print_foot_pos();

  dog.updateDogState();
  dog.getDogState(&dogState);
  cout << "pos ";
  SD_VEC_PRINT(dogState.pos);
  sim->getBodyInformation(LittleDog::B_FL_LLEG, &pos, &qOri);
  cout << "leg pos ";
  SD_VEC_PRINT(pos);
  cout << endl << endl;
}

int main(int argc, char** argv) {
  init();
  test_cross_leg_fall();
  //test_one_leg_move();
  //test_all_legs_move();
  return 0;
}
