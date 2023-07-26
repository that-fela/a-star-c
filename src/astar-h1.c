#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "astar-h1.h"

extern const unsigned char map[MAP_HEIGHT][MAP_WIDTH];

typedef struct {
    Point point;
    int fScore;
    int gScore;
} Node;

int heuristic(Point p1, Point p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y); 
}

bool isValidPoint(Point point) {
    return point.x >= 0 && point.x < MAP_WIDTH && point.y >= 0 && point.y < MAP_HEIGHT; 
}

bool isObstacle(Point point) {
    return map[point.y][point.x] == 1; 
}

void reconstructPath(Point cameFrom[MAP_HEIGHT][MAP_WIDTH], Point current, RobotDir* directions) {
    Point path[MAP_WIDTH * MAP_HEIGHT];
    int pathLength = 0;
    while (current.x != -1 && current.y != -1) {
        path[pathLength++] = current;
        current = cameFrom[current.y][current.x];
    }

    printf("Path: ");
    
    MoveDir currentDir;
    MoveDir previousDir;
    for (int i = pathLength - 2; i >= 0; i--) {
        printf("(%d, %d) ", path[i].x, path[i].y); 

        int deltaX = path[i].x - path[i + 1].x; 
        int deltaY = path[i].y - path[i + 1].y; 

        if (deltaX > 0) {
            currentDir.orientation = RIGHT;
        } else if (deltaX < 0) {
            currentDir.orientation = LEFT;
        } else if (deltaY > 0) {
            currentDir.orientation = DOWN;
        } else if (deltaY < 0) {
            currentDir.orientation = UP;
        }

        currentDir.amount = 1;

        if (currentDir.orientation == previousDir.orientation) {
            directions->moveDirs[directions->length - 1].amount++;
        } else {
            directions->moveDirs[directions->length++] = currentDir;
        }

        previousDir = currentDir;
    }
    printf("\n");
}

// A* algorithm
void aStarSearch(Point start, Point goal, RobotDir* directions) {
    bool openSet[MAP_HEIGHT][MAP_WIDTH];   
    Point cameFrom[MAP_HEIGHT][MAP_WIDTH]; 

    for (int i = 0; i < MAP_HEIGHT; i++) {
        for (int j = 0; j < MAP_WIDTH; j++) {
            openSet[i][j] = false;
            cameFrom[i][j].x = -1;
            cameFrom[i][j].y = -1;
        }
    }

    int gScore[MAP_HEIGHT][MAP_WIDTH];
    int fScore[MAP_HEIGHT][MAP_WIDTH];
    for (int i = 0; i < MAP_HEIGHT; i++) {
        for (int j = 0; j < MAP_WIDTH; j++) {
            gScore[i][j] = INFINITY;
            fScore[i][j] = INFINITY;
        }
    }

    gScore[start.y][start.x] = 0; 
    fScore[start.y][start.x] = heuristic(start, goal); 

    openSet[start.y][start.x] = true; 

    int count;
    while (true) {
        int minFScore = INFINITY;
        Point current;

        if (count++ > 100) {
            printf("Error: count exceeded 10000\n");
            break;
        }

        for (int i = 0; i < MAP_HEIGHT; i++) {
            for (int j = 0; j < MAP_WIDTH; j++) {
                if (openSet[i][j] && fScore[i][j] < minFScore) {
                    minFScore = fScore[i][j];
                    current.y = i; 
                    current.x = j; 
                }
            }
        }

        if (current.y == goal.y && current.x == goal.x) { 
            reconstructPath(cameFrom, current, directions);
            printf("Completed in %d steps\n", count);
            break;
        }

        openSet[current.y][current.x] = false; 

        Point neighbors[4] = {
            {current.x, current.y - 1}, 
            {current.x, current.y + 1}, 
            {current.x - 1, current.y}, 
            {current.x + 1, current.y}  
        };

        for (int i = 0; i < 4; i++) {
            Point neighbor = neighbors[i];
            if (!isValidPoint(neighbor) || isObstacle(neighbor)) {
                continue;
            }

            int tentativeGScore = gScore[current.y][current.x] + 1; 

            if (tentativeGScore < gScore[neighbor.y][neighbor.x]) { 
                cameFrom[neighbor.y][neighbor.x] = current; 
                gScore[neighbor.y][neighbor.x] = tentativeGScore; 
                fScore[neighbor.y][neighbor.x] = tentativeGScore + heuristic(neighbor, goal); 

                if (!openSet[neighbor.y][neighbor.x]) { 
                    openSet[neighbor.y][neighbor.x] = true; 
                }
            }
        }
    }
}

void printRobotDirections(RobotDir* directions) {
    printf("Robot Directions:\n");
    for (int i = 0; i < directions->length; i++) {
        if (directions->moveDirs[i].orientation == 0) {
            printf("UP");
        } else if (directions->moveDirs[i].orientation == 1) {
            printf("DOWN");
        } else if (directions->moveDirs[i].orientation == 2) {
            printf("LEFT");
        } else if (directions->moveDirs[i].orientation == 3) {
            printf("RIGHT");
        }

        printf("\t%d\n", directions->moveDirs[i].amount);
    }
}