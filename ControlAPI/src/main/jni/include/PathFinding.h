//
// Created by gaujei on 10/30/15.
//

#ifndef ASUSBOT_PATHFINDING_H
#define ASUSBOT_PATHFINDING_H

#include  <string.h>


#define Android
#define DebugMode	// Some necessary information will be recorded in log.txt and path.txt


typedef struct {
    int sizeSeg;               // size of the route
    int relPathPt[2][256];  // each segment of the calculated path
}_ROUTE;

class node {
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;	// smaller: higher priority
    node() {}		// priviate default constructor

public:
    node(int xp, int yp, int d, int p);	// node(int xp, int yp, int d, int p)	{xPos=xp; yPos=yp; level=d; priority=p;}

    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}

    // A*
    void updatePriority(const int & xDest, const int & yDest);

    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i);

    // Estimation function for the remaining distance to the goal.
    const int & estimate(const int & xDest, const int & yDest);
};

// const char * pathFind( int xStart, int yStart, int xFinish, int yFinish );
// _ROUTE pathFind( int init[], int target[] );
_ROUTE pathFind( int init[], int target[] );


#endif //ASUSBOT_PATHFINDING_H
