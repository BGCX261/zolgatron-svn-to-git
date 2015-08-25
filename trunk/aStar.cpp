/*
 * aStar.cpp
 *
 *  Created on: Oct 20, 2009
 *      Author: nikhil
 */

#include "aStar.h"
#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDVectorOp.h"
#include "Classifier.h"

aStar::aStar() {
	// TODO Auto-generated contructor stub
}

aStar::~aStar() {
	// TODO Auto-generated destructor stub
}


bool aStar::tryMove(int foot, int grid_position, SDSimulatorState *currentState, queueNode node) {
	double x = node.location.foot[foot].n[0] + (grid_position % grid_size_ - 3) * stepSize_;
	double y = node.location.foot[foot].n[1] + (grid_position / grid_size_ - 3) * stepSize_;

	walkController_->setStepPosition(foot, x, y);

	// TODO generate features for the classifier here
	littleDog_->runTrial(currentState);
	simulator_->getState(currentState);

	bduVec3f globalFeet[4];
	simulator_->getGlobalFootPositions(globalFeet);

	return ifStepSuccess(x, y, globalFeet[foot]);
}

void aStar::search() {
	priority_queue<queueNode, vector<queueNode> , queueNode> Q;

	// get starting feet positions
	bduVec3f globalFeet[4];
	simulator_->getGlobalFootPositions(globalFeet);

	// Set starting feet positions in the queue element
	queueNode firstNode;
	SD_VEC_COPY(firstNode.location.foot[0], globalFeet[0]);
	SD_VEC_COPY(firstNode.location.foot[1], globalFeet[1]);
	SD_VEC_COPY(firstNode.location.foot[2], globalFeet[2]);
	SD_VEC_COPY(firstNode.location.foot[3], globalFeet[3]);

	firstNode.numSteps = 0;
	firstNode.valHeuristic = 2.75;

	simulator_->getState(&firstNode.state);
	Q.push(firstNode);

	while (!Q.empty()){

		queueNode poppedNode = Q.top();
		Q.pop();

		for (int leg_0_grid = 1; leg_0_grid <= grid_size_ * grid_size_; leg_0_grid++) {
			// set dog's state to popped poppedNodes state
			simulator_->setState(&poppedNode.state);

			// and save it to currentState - this will be reset to original popped state when all inner loops complete
			SDSimulatorState currentState;
			simulator_->getState(&currentState);

			if (tryMove(0, leg_0_grid, &currentState, poppedNode)) {

				for (int leg_1_grid = 1; leg_1_grid <= grid_size_; leg_1_grid++) {
					if (tryMove(1, leg_1_grid, &currentState, poppedNode)) {

						for (int leg_3_grid = 1; leg_3_grid <= grid_size_; leg_3_grid++) {
							if (tryMove(3, leg_3_grid, &currentState, poppedNode)) {

								for (int leg_2_grid = 1; leg_2_grid <= grid_size_; leg_2_grid++) {
									if (tryMove(2, leg_2_grid, &currentState, poppedNode)) {

										//	cout << "INSIDE......"<<endl;

										// All legs moved successfully, add to the priority queue
										queueNode newNode;

										bduVec3f globalFeet[4];
										simulator_->getGlobalFootPositions(globalFeet);

										SD_VEC_COPY(newNode.location.foot[0], globalFeet[0]);
										SD_VEC_COPY(newNode.location.foot[1], globalFeet[1]);
										SD_VEC_COPY(newNode.location.foot[2], globalFeet[2]);
										SD_VEC_COPY(newNode.location.foot[3], globalFeet[3]);
                                        SD_VEC_PRINT(globalFeet[0]);
                                        SD_VEC_PRINT(globalFeet[1]);
                                        SD_VEC_PRINT(globalFeet[2]);
                                        SD_VEC_PRINT(globalFeet[3]);

										newNode.numSteps = poppedNode.numSteps + 4;
										simulator_->getState(&newNode.state);
										newNode.valHeuristic = calcHeuristic(newNode);
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

double aStar::calcHeuristic(queueNode node) {
	bduVec3f goalPos;
	goalPos.n[0] = goal_x_;
	goalPos.n[1] = goal_y_;
	goalPos.n[2] = 0.0;

	double heuristic = 0.0;
	heuristic += SD_VEC_DIST(goalPos, node.location.foot[0]);
	heuristic += SD_VEC_DIST(goalPos, node.location.foot[1]);
	heuristic += SD_VEC_DIST(goalPos, node.location.foot[2]);
	heuristic += SD_VEC_DIST(goalPos, node.location.foot[3]);

	heuristic /= 4;
	heuristic = 4*heuristic/stepSize_;
	return heuristic;
}

void aStar::setGridSize(int size) {
	grid_size_ = size;
}

void aStar::setSimulator(SDSimulator *simulator) {
	simulator_ = simulator;
}

void aStar::setLittleDog(SDLittleDog *littleDog) {
	littleDog_ = littleDog;
}

void aStar::setStepSize(double stepSize) {
	stepSize_ = stepSize;
}

void aStar::setWalkController(SDSlowWalk *walkController) {
	walkController_ = walkController;
}

void aStar::setGoal(double goal_x, double goal_y) {
	goal_x_ = goal_x;
	goal_y_ = goal_y;
}
