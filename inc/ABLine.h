#ifndef _ABLine_H_
#define _ABLine_H_ 

#include "vehicle.h"
#include "glm.h"
#include "nmea.h"
#include <math.h>

typedef struct {
    int isABLineSet;        //bool
    int isABSameAsVehicleHeading;   //bool
    vec2 refPoint1;
    vec2 refPoint2;
    vec2 refABLineP1;
    vec2 refABLineP2;
    double abHeading;
    double distanceFromRefLine;
    double distanceFromCurrentLine;
    double refLineSide;
    int howManyPathsAway;
    double widthMinusOverlap;
    double passNumber;
} ABline;
void btnAPoint(void);
void btnBPoint(void);
void SetABLineByHeading(void);
void MoveABLine(double dist);
void initABl(void);
#endif

