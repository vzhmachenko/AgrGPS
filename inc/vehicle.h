#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "stm32f4xx.h"


struct Vehicle{
    double toolWidth;
    double toolOverlap;
    double toolOffset;

    double antennaHeight;
    double antennaOffset;
    double antennaPivot;

    double wheelbase;
    double hitchLength;
    double minTurningRadius;
};

typedef struct {
    double easting;
    double northing;
} vec2;

typedef struct {
    double easting;
    double northing;
    double heading;
} vec3;

typedef struct {
      //very first fix to setup grid etc
        int8_t isFirstFixPositionSet, isGPSPositionInitialized;

        // autosteer variables for sending serial
        int16_t guidanceLineDistanceOff, guidanceLineSteerAngle;

        //how far between new section triangle trigger
        double triangleResolution;

        //how many fix updates per sec
        //int fixUpdateHz = 5;
        //double fixUpdateTime = 0.2;

        //for heading or Atan2 as camera
        //string headingFromSource;

        vec3 pivotAxlePos;;
        vec3 toolPos;
        vec3 tankPos;
        vec2 hitchPos;

        //history
        vec2 prevFix;

        //headings
        double fixHeading, camHeading, gpsHeading, prevGPSHeading;

        //storage for the cos and sin of heading
        double cosSectionHeading, sinSectionHeading;

        //a distance between previous and current fix
        double distance;
  
        //how far travelled since last section was added, section points
        double sectionTriggerDistance, sectionTriggerStepDistance; 
        vec2 prevSectionPos;
        
        //step distances and positions for boundary, 4 meters before next point
        double boundaryTriggerDistance;
        vec2 prevBoundaryPos;

        //are we still getting valid data from GPS, resets to 0 in NMEA OGI block, watchdog 
        int recvCounter;

        //Everything is so wonky at the start
        int startCounter;


        //tally counters for display
        //double totalSquareMetersWorked = 0, totalUserSquareMeters = 0, userSquareMetersAlarm = 0;
/*
        double[] avgSpeed = new double[10];//for average speed
        double[] avgXTE = new double[20]; //for average cross track error
        int ringCounter = 0, avgXTECntr, crossTrackError;

        //youturn
        double distancePivotToTurnLine = -2222;
        double distanceToolToTurnLine = -2222;
        
        //the value to fill in you turn progress bar
        int youTurnProgressBar = 0;

        //IMU 
        double rollCorrectionDistance = 0;
        double gyroDelta, gyroCorrection, gyroRaw, gyroCorrected;

        //step position - slow speed spinner killer
        private int totalFixSteps = 10, currentStepFix = 0;
        private vec3 vHold;
        vec3[] stepFixPts = new vec3[60];
        double distanceCurrentStepFix = 0, fixStepDist, minFixStepDist = 0;        
        bool isFixHolding = false, isFixHoldLoaded = false;
  */      
} mainForm;
 #endif
