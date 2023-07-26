#ifndef _ASTAR_H1_H_
#define _ASTAR_H1_H_

#ifndef MAP_WIDTH
#define MAP_WIDTH 19
#endif // MAP_WIDTH

#ifndef MAP_HEIGHT
#define MAP_HEIGHT 15
#endif // MAP_HEIGHT

#ifndef DIRECTIONS_MAX
#define DIRECTIONS_MAX 100
#endif // DIRECTIONS_MAX

typedef struct {
    int x;
    int y;
} Point;

// Robot directions

typedef enum {
    UP,
    DOWN,
    LEFT,
    RIGHT,
} Orientation;

typedef struct {
    Orientation orientation;
    unsigned int amount;
} MoveDir;

typedef struct {
    int length;
    MoveDir moveDirs[DIRECTIONS_MAX];
} RobotDir;

// API

void aStarSearch(Point start, Point goal, RobotDir* directions);

void printRobotDirections(RobotDir* directions);

#endif // _ASTAR_H1_H_
