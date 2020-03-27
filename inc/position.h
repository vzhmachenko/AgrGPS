#ifndef _POSITION_H_
#define _POSITION_H_

//#include "nmea.h"
#include <math.h>
#include "stm32f4xx.h"
#include "glm.h"

#define histSize 6

enum {
  isFirstFixPositionSet     = 0,
  isGPSPositionInitialized  = 1,
  isFixHolding              = 2,
  isFixHoldLoaded           = 3
};

typedef struct {

  int     isFirstFixPositionSet;      // <bool> ///< Самая первая фиксация точки и т.д.
  int     isGPSPositionInitialized;   // <bool> ///< Связь устоявшаяся


  int16_t guidanceLineDistanceOff;
  int16_t guidanceLineSteerAngle;

  double  distanceCurrentStepFix;
  double  fixStepDist;
  double  minFixStepDist;
  double  distance;
  double  boundaryTriggerDistance;
  double  fixHeading;             // "Историческое" направление

  int offset;   // Смещение в сантиметрах

  vec2    prevFix;
  vec2    prevBoundaryPos;

  vec3    stepFixPts[histSize];
  vec3    stepFixPts_last;
  vec3    pivotAxlePos;
  vec3    vHold;

  uint8_t flags;


} Position;

void initPosition(void);
void UpdateFixPosition(void);
void CalculatePositionHeading(void);
void InitializeFirstFewGPSPositions(void);
/* ---------------------------------------------------------- */
 #endif
