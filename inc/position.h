#ifndef _POSITION_H_
#define _POSITION_H_

#include "nmea.h"
#include <math.h>
#include "glm.h"
typedef struct {

  int     isFirstFixPositionSet;      // <bool> ///< Самая первая фиксация точки и т.д.
  int     isGPSPositionInitialized;   // <bool> ///< Связь устоявшаяся

  int     isFixHolding;               // <bool>
  int     isFixHoldLoaded;            // <bool>
  int     startCounter;               // Начальный счетчик
  int     totalFixSteps;
  int     fixUpdateHz;
  int     currentStepFix;

  int16_t guidanceLineDistanceOff;
  int16_t guidanceLineSteerAngle;

  double  distanceCurrentStepFix;
  double  fixStepDist;
  double  minFixStepDist;
  double  distance;
  double  boundaryTriggerDistance;
  double  fixHeading;

  vec2    prevFix;
  vec2    prevBoundaryPos;

  vec3    stepFixPts[60];
  vec3    pivotAxlePos;
  vec3    vHold;


} position;

void initPosition(void);
void UpdateFixPosition(void);
void CalculatePositionHeading(void);

 #endif
