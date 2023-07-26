#include <stdio.h>
#include <stdlib.h>

#include "src/map.h"
#include "src/astar-h1.h"

#define MAP_HEIGHT 15
#define MAP_WIDTH 19

extern const unsigned char map[MAP_HEIGHT][MAP_WIDTH];

int main() {

    Point start = {13, 3};
    Point goal = {6, 9};

    RobotDir directions = {0};

    aStarSearch(start, goal, &directions);

    printRobotDirections(&directions);

    return 0;
}
