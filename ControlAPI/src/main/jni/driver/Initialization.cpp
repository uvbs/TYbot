//
// Created by gaujei on 11/2/15.
//

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <stdio.h> //printf
#include <stdlib.h>
#include "../include/DebugMessage.h"

// Added by Ting-Ying
#include <iomanip>
#include <queue>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>

#include "include/asusbot.h"
#include "include/PathFinding.h"
#include "include/Initialization.h"

#include "DriverInterface.h"
#include "asusbotmainloop.h"

//#include <android/log.h>
#define LOG_TAG  "ASUS_LOAD_FILES"

int SpecificActionTable[][3];


//
int loadSpecificAction(const char* filepath ){
    // Load the map from a grip map file, map.txt, in which the number '1' indicates an obstacle and '0' is free space
    std::vector< std::vector<int> > table = readIn2dData( filepath );
    // Convert the vector table to a 2D array
    int m = table.size();			// vertical size of the map (y-axis)
    int n = table[0].size();		// horizontal size of the map (x-axis)

    LOGD("table.size() = %d and table[0].size() = %d", m, n);

    // fill in the data from vector::table[height][width]
    for (int y = 0; y < m; ++y) {
        for (int x = 0; x < n; ++x) {
            SpecificActionTable[x][y] = table[y][x];
            // cout << SpecificActionTable[x][y] << " ";
        }
        // cout << endl;
    }
    return 0;
}


void writeTextToFile(const char* filepath, int Width, int Height) {
    static int value = 0;

    FILE* file = fopen(filepath, "w+b");
    if (file != NULL)
    {
        fputs("# Width = 100; Height = 100 \n", file);
        for (int y = 1; y <= Height; y++) {
            for (int x = 1; x <= Width; x++) {
                if ((x > Width * 3 / 8 && x < Width * 4 / 8 && y >= Height * 1 / 8 && y <= Height * 6 / 8) ||
                    (x > Width * 5 / 8 && x < Width * 6 / 8 && y >= Height * 3 / 8 && y <= Height * 8 / 8)) {
                    fputc('1', file);
                }
                else {
                    fputc('0', file);
                }
                fputc(' ', file);
            }
            fputc('\n', file);
        }
        fflush(file);
        fclose(file);
#ifdef DebugMode
#ifdef Android
        LOGI("WriteTextToFile success!");
#else	// Visual
        cout << "WriteTextToFile success! " << endl;
#endif
#endif
    }
    else {
#ifdef DebugMode
#ifdef Android
        LOGI("WriteTextToFile Fail!");
#else	// Visual
        cout << "WriteTextToFile Fail! " << endl;
#endif
#endif
    }

}

/*
 * Function takes a char* filename argument and returns a 2D dynamic array containing the data
*/
std::vector< std::vector<int> > readIn2dData(const char* filename) {

    std::vector< std::vector<int> > table;
    std::fstream ifs;

    /*  open file  */
    ifs.open(filename);
    if (!ifs.is_open()) {
#ifdef DebugMode
#ifdef Android
        LOGI("Failed to open the map file: /sdcard/map.txt!");
#else	// Visual
        cout << "Failed to open the map file: /sdcard/map.txt!" << endl;
#endif
#endif
    }
    else {
        while (true) {
            std::vector<int> row;	// Reset the value of row at each cycle
            std::string line;
            int buf;

            getline(ifs, line);
            std::stringstream ss(line, std::ios_base::in | std::ios_base::out | std::ios_base::binary);
            if (!ifs)								// mainly catch EOF
                break;
            if (line[0] == '#' || line.empty())		// catch empty lines or comment lines
                continue;

            // Assign the row and column values to "table"
            while (ss >> buf)
                row.push_back(buf);
            table.push_back(row);
        }
        ifs.close();
    }

    return table;
}
