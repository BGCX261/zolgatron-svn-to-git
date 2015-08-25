/*
 * aStar.h
 *
 *  Created on: Oct 20, 2009
 *      Author: nikhil
 */

#include<queue>
#include "SDLittleDog.h"
#include "SDSlowWalk.h"
#include "SDVectorOp.h"
#include "Classifier.h"

#ifndef ASTAR_H_
#define ASTAR_H_

class aStar {
public:

	class queueNode {
	public:
		struct location_quad {
			bduVec3f foot[4];
		};
		location_quad location;
		// TODO keep the path till now
		// TODO check for repeated positions within some limites
		//	list<location_quad> path;
		SDSimulatorState state;
		double numSteps;
		double valHeuristic;

		// Returns true if G + H value for qe1 is < qe 2
		bool operator()(queueNode& qe1, queueNode& qe2) {
			if (qe1.numSteps + qe1.valHeuristic < qe2.numSteps + qe2.valHeuristic) {
				return true;
			}
			return false;
		}
	};

	aStar();
	virtual ~aStar();

	bool tryMove(int foot, int grid_position, SDSimulatorState *currentState, queueNode node);
	void setGridSize(int size);
	void setSimulator(SDSimulator *simulator);
	void setLittleDog(SDLittleDog *littleDog);
	void setWalkController(SDSlowWalk *walkController);
	void setStepSize(double stepSize);
	void setGoal(double goal_x, double goal_y);
	void search();
	double calcHeuristic(queueNode node);

private:
	int grid_size_;
	SDSimulator *simulator_;
	SDLittleDog *littleDog_;
	SDSlowWalk *walkController_;
	double stepSize_;
	double goal_x_;
	double goal_y_;

	bool ifStepSuccess( double x, double y, bduVec3f globalFeet)
	{
	  double dist =
	  sqrt((x-globalFeet.n[0])*(x-globalFeet.n[0])+((y-globalFeet.n[1])*(y-globalFeet.n[1])));
	  if( dist<0.01)
	    return true;
	  else
	    return false;
	}
};

#endif /* ASTAR_H_ */
