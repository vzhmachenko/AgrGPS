#ifndef _NMEA_H_
#define _NMEA_H_

#include "stm_tasks.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>


enum { //flags
  zone          = 0,
  latitudeOk    = 1,
  longtitudeOk  = 2
};

typedef struct {
  //WGS84 Lat Long
  double latitude;                              ///< Широта
  double longitude;                             ///< Долгота
  double actualEasting, actualNorthing;
  double zone;                                  ///< UTM зона
  double centralMeridian;                       ///< Центральный меридиан текущей зоны 
  double convergenceAngle;                      ///< Ошибка азимута, utm-склонение


  uint8_t updatedGGA, updatedOGI, updatedRMC;

  char *rawBuffer;
  //-----------------------------------------------------
  char time[6]; ///< time buffer
  char date[6]; ///< date buffer
  //-----------------------------------------------------
  //UTM coordinates
  //double northing, easting;
  vec2 fix;   //0,0
  vec2 prevFix;

  //used to offset the antenna position to compensate for drift
  vec2 fixOffset;     //0,0

  //other GIS Info
  double altitude;                        ///< Высота приемника (антенны) над уровнем моря
  double speed;                           ///< Скорость, в кнотах
  double headingTrue; 
  double ageDiff;                         ///< Различия в "элипсоидах"
  double hdop;                            ///< Точность позиционирования по горизонтали 

  //imu
  double nRoll, nPitch, nYaw, nAngularVelocity;
  uint8_t isValidIMU;
  int16_t fixQuality;                     ///< Качество позиционирования
  int16_t satellitesTracked;              ///< Количество захваченных спутников
  char status;
  char utcDateTime[15];
  char hemisphere;

  //UTM numbers are huge, these cut them way down.
  int32_t utmNorth, utmEast;

  //vec3 stepFixPts[50];

  uint8_t flags;

} NMEA;  


void    createStartNMEA (void);
double  ArcLengthOfMeridian(double phi);
void    MapLatLonToXY(double phi, double lambda, double lambda0);
void    DecDeg2UTM(double latitude, double longitude);
void    UpdateNorthingEasting(void);
void    splitString(char* from);
double  NMEAtoDecimal(char* str);
void    ParseGGA(void);
void    ParseGLL(void);
void    ParseRMC(void);
void    ParseVTG(void); 
void    checkLatLon(char* lat, char* lon);

/* ---------------------------------------------------- */
 #endif