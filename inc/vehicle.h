#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "stm32f4xx.h"


typedef struct {
  double toolWidth;
  double toolOverlap;
  double toolOffset;

  double antennaHeight;
  int antennaOffset; // Смещение в сантиметрах
  double antennaPivot;
  double maxSteerAngle;
  double maxAngularVelocity;

  double wheelbase;
  double hitchLength;
  double minTurningRadius;

  double goalPointLookAheadSeconds;// = 3.0;
  double goalPointDistanceMultiplier;// = 0.1;
  double goalPointLookAheadMinimumDistance;// = 2;    
}Vehicle;

typedef struct {
  double easting;
  double northing;
} vec2;

typedef struct {
  double easting;
  double northing;
  double heading;
} vec3;

double UpdateGoalPointDistance(double distanceFromCurrentLine);
void   initVehicle(void);

 #endif
