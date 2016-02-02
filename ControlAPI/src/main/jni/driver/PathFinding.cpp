//
// Created by gaujei on 10/30/15.
//
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <stdio.h> //printf
#include <stdlib.h>

// Added by Ting-Ying
#include <iomanip>
#include <queue>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>

#include "include/asusbot.h"
#include "include/Trajectory.h"
#include "include/command.h"
#include "include/Threads.h"
#include "include/PathFinding.h"
#include "include/Initialization.h"
#include "include/DebugMessage.h"

#include "DriverInterface.h"
#include "asusbotmainloop.h"

//#include <android/log.h>
#define LOG_TAG  "ASUS_PATH_FINDING"

extern std::vector< std::vector<int> > Table;
_ROUTE Route;


// Functions for the path-finding algorithm
node::node(int xp, int yp, int d, int p) {
    xPos=xp; yPos=yp; level=d; priority=p;
}

void node::updatePriority(const int & xDest, const int & yDest) {
    priority = level + estimate(xDest, yDest)*10; //A*
}

// give better priority to going strait instead of diagonally
void node::nextLevel(const int & i) {	// i: direction
    level += (i%2==0? 10:14);
}


// Estimation function for the remaining distance to the goal.
const int & node::estimate(const int & xDest, const int & yDest) {
    static int xd, yd, d;
    xd = xDest-xPos;
    yd = yDest-yPos;

    // Euclidian Distance
    //d = static_cast<int>(sqrt(float (xd*xd+yd*yd)));

    // Manhattan distance
    d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return(d);
}

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b) {
    return a.getPriority() > b.getPriority();
}



// Added by Ting-Ying
// A-star algorithm : The route returned is a string of direction digits.
_ROUTE pathFind(int initPose[], int targetPose[]) {
    // if dir==8
#define dir 8	// number of possible directions to go at any position

    int xStart = initPose[0], yStart = initPose[1];		// thetaStart = initPose[2];
    int xFinish = targetPose[0], yFinish = targetPose[1];	// thetaFinish = targetPose[2];
    int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
    int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};
    static std::priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    int gridPath[512], gridPoint = 0;
    pqi = 0;


    // Load the map from a grip map file, map.txt, in which the number '1' indicates an obstacle and '0' is free space
    // std::vector< std::vector<int> > Table = readIn2dData( _FilePath );
    // Convert the vector table to a 2D array
    int m = Table.size();			// vertical size of the map (y-axis)
    int n = Table[0].size();		// horizontal size of the map (x-axis)

    // Allocate dynamic 2d map[width][height]
    int ** map = new int*[n];
    int ** dir_map = new int*[n];			// dir_map[n][m] : map of directions
    int ** open_nodes_map = new int*[n];	// open_nodes_map[n][m] : map of open (not-yet-tried) nodes
    int ** closed_nodes_map = new int*[n];	// closed_nodes_map[n][m] : map of closed (tried-out) nodes
    for (int x = 0; x < n; ++x) {			// n = width
        map[x] = new int[m];				// m = height
        dir_map[x] = new int[m];
        open_nodes_map[x] = new int[m];
        closed_nodes_map[x] = new int[m];
    }
    // fill in the data from vector::table[height][width]
    for (int y = 0; y < m; ++y) {
        for (int x = 0; x < n; ++x) {
            map[x][y] = Table[y][x];	// int map[n][m];
            dir_map[x][y] = 0;
            open_nodes_map[x][y] = 0;
            closed_nodes_map[x][y] = 0;
            // cout << closed_nodes_map[x][y] << " ";
        }
        // cout << endl;
    }

    // create the start node and push into list of open nodes
    n0 = new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y] = n0->getPriority(); // mark it on the open nodes map

    // A* search
    while (!pq[pqi].empty())
    {
        // Get the current node w/ the highest priority from the list of open nodes
        n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                      pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x = n0->getxPos();
        y = n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y] = 0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y] = 1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if (x == xFinish && y == yFinish)
        {
            // Extract the path from finish to start by following the directions
            std::string path = "";
            while (!(x == xStart && y == yStart))
            {
                j = dir_map[x][y];
                c = '0' + (j + dir / 2) % dir;	// Change forward direction as backward direction
                path = c + path;
                x += dx[j];
                y += dy[j];
                gridPath[gridPoint] = (j + dir / 2) % dir;
                gridPoint++;
            }

            // Route.size, Route.relPathPt
            int grid = 1, seg = 1;	// How many grid in current segment
            int * relPathPt = new int[2 * gridPoint];		// relative path in x- and y-axis (unit: grid point)
            for (int i = (gridPoint - 2); i >= 0; i--) {
                if (i == 0 || (gridPath[i] != gridPath[i + 1])) {
                    if (seg == 1) {
                        Route.relPathPt[0][0] = grid * dx[gridPath[i + 1]];	// direction along x-axis direction
                        Route.relPathPt[1][0] = grid * dy[gridPath[i + 1]];	// direction along y-axis direction
                    }
                    else {
                        Route.relPathPt[0][seg - 1] = grid * dx[gridPath[i + 1]] + Route.relPathPt[0][seg - 2];
                        Route.relPathPt[1][seg - 1] = grid * dy[gridPath[i + 1]] + Route.relPathPt[1][seg - 2];		// direction along x-axis direction
                    }
                    grid = 1;
                    seg++;
                }
                else {	// ( gridPath[i] == gridPath[i+1]	)
                    grid++;
                }
            }
            Route.relPathPt[0][seg - 1] = xFinish - xStart;
            Route.relPathPt[1][seg - 1] = yFinish - yStart;
            Route.sizeSeg = seg;

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while (!pq[pqi].empty()) pq[pqi].pop();

#ifdef DebugMode
            std::ofstream LogFile;
            LogFile.open( _LogFile, std::ios::out | std::ios::app); //write and append the data
            if (LogFile.is_open())
            {
                LogFile << path.data() << "\n";
                LogFile.close();
            }

            std::ofstream LogPath;
            LogPath.open( _LogPath );
            if (LogPath.is_open())
            {
				for (int i = 0; i < Route.sizeSeg; i++) {
					LogPath << Route.relPathPt[0][i] << " ";
					LogPath << Route.relPathPt[1][i] << "\n";
				}
                LogPath.close();
            }
#endif
            return Route; // return (path.c_str());
        }

        // generate moves (child nodes) in all possible directions
        for( i=0; i<dir; i++ )
        {
            xdx = x + dx[i];
            ydy = y + dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1
                 || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                            pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    // Garbage collection
    for (int i = 0; i < n; i++) {
        delete[] map[i];
        delete[] dir_map[i];			// dir_map[n][m] : map of directions
        delete[] open_nodes_map[i];		// open_nodes_map[n][m] : map of open (not-yet-tried) nodes
        delete[] closed_nodes_map[i];	// closed_nodes_map[n][m] : map of closed (tried-out) nodes
    }
    delete[] map;
    delete[] dir_map;
    delete[] open_nodes_map;
    delete[] closed_nodes_map;

    // std::string path = "No route found!";
    return Route;	// return (path.c_str()); // no route found
}
