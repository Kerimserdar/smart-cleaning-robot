#ifndef _FUNCTION_H   
#define _FUNCTION_H  

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>  
#include <time.h>

struct Point
{
	int x;
	int y;
	bool active; // this is for possibilty list
	char mark; // if it is marked as "o" which means it is obstacle by obstacle or if it is marked by "t" which means that it is marked by trace list 
};

typedef struct {
  struct Point *array;
  size_t used;
  size_t size;
} Array;

struct Point* createPosList(struct Point currentLocation);
struct Point randomSelection(struct Point *posList);

extern int direction;

void initArray(Array *a, size_t initialSize);
void insertArray(Array *a, struct Point point);
void freeArray(Array *a);
int printPointList(struct Point *list, int size);
void checkObstacleList(struct Point *posList, Array obstacle);
void checkTraceList(struct Point *posList, Array trace);
int checkDeviceSurrounded(struct Point *posList);
int getInfoDistanceSensor(int size);
int detectDirection(struct Point selectedPoint, struct Point currentLocation, int direction);
void initialize(void);
void recursive(int val);

#endif 
