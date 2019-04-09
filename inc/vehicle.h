#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "stm32f4xx.h"


typedef struct {
    double toolWidth;
    double toolOverlap;
    double toolOffset;

    double antennaHeight;
    double antennaOffset;
    double antennaPivot;

    double wheelbase;
    double hitchLength;
    double minTurningRadius;
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


 #endif
