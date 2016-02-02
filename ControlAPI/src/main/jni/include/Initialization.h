//
// Created by gaujei on 11/2/15.
//

#ifndef ASUSBOT_INITIALIZATION_H
#define ASUSBOT_INITIALIZATION_H

#define _Specific   "./sdcard/SpecificAction.txt"
#define _FilePath	"./sdcard/map.txt"
#define _LogFile	"./sdcard/log.txt"
#define _LogPath	"./sdcard/path.txt"


// Declaration for the extern values
extern int SpecificActionTable[30][3];


// Declaration for the functions and class
int loadSpecificAction(const char* filename);

void writeTextToFile(const char * filepath, int Width, int Height);

std::vector< std::vector<int> > readIn2dData(const char* filename);


#endif //ASUSBOT_INITIALIZATION_H
