#ifndef _POSITION_H_
#define _POSITION_H_

#include "nmea.h"
#include <math.h>
#include "glm.h"
typedef struct {
    
        int isGPSPositionInitialized;   //bool
        int isFirstFixPositionSet;      //bool
        int isFixHolding;               //bool
        int isFixHoldLoaded;           //bool
        int startCounter;
        int totalFixSteps;
        int fixUpdateHz;
        double distanceCurrentStepFix;
        double fixStepDist;
        double minFixStepDist;
        double distance;
        double boundaryTriggerDistance;
        double fixHeading;
        int currentStepFix;
        vec2 prevFix;
        vec2 prevBoundaryPos;
        vec3 stepFixPts[60];
        vec3 pivotAxlePos;
        vec3 vHold;
        int16_t guidanceLineDistanceOff, guidanceLineSteerAngle;

} position;

void initPosition(void);
void UpdateFixPosition(void);
void CalculatePositionHeading(void);

 #endif
