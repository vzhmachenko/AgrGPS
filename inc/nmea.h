#ifndef _NMEA_H_
#define _NMEA_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vehicle.h"
#include "math.h"
typedef struct {
     //WGS84 Lat Long
        double latitude, longitude;
        double actualEasting, actualNorthing;
        double zone;
        double centralMeridian, convergenceAngle;

        uint8_t updatedGGA, updatedOGI, updatedRMC;

        char *rawBuffer;
        char time[6];
        char date[6];
        //UTM coordinates
        //double northing, easting;
        vec2 fix;   //0,0
        //used to offset the antenna position to compensate for drift
        vec2 fixOffset;     //0,0

        //other GIS Info
        double altitude, speed;
        double headingTrue, headingHDT, hdop, ageDiff;

        //imu
        double nRoll, nPitch, nYaw, nAngularVelocity;
        uint8_t isValidIMU;
        int16_t fixQuality;
        int16_t satellitesTracked;
        char status;
        char utcDateTime[15];
        char hemisphere;

        //UTM numbers are huge, these cut them way down.
        int16_t utmNorth, utmEast;

} NMEA;  


void createStartNMEA (void);
double ArcLengthOfMeridian(double phi);
void MapLatLonToXY(double phi, double lambda, double lambda0);
void DecDeg2UTM(double latitude, double longitude);
void UpdateNorthingEasting(void);
void ParseNMEA(void *parameter);

 #endif
