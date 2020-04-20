#ifndef _ABLine_H_
#define _ABLine_H_ 

#include <math.h>
#include "gpio.h"
#include "stm_tasks.h"

enum {
  APointSet                = 0,
  BPointSet                = 1,
  ABLineSet                = 2,
  isABSameAsVehicleHeading = 3,
  isOnRightSideCurrentLine = 4
};

typedef struct {
  vec2 refPoint1;                 //Установленная точка А
  vec2 refPoint2;                 //Установленная точка В
  vec2 refABLineP1;               //Пересчитан конец точки А
  vec2 refABLineP2;               //Пересчитан конец точки В
  vec2 currentABLineP1;           //Точка А линии, по которой движемся
  vec2 currentABLineP2;           //Точка B линии, по которой движемся
  vec2 goalPointAB;
  vec2 radiusPointAB;

  int8_t refLineSide;
  int8_t passNumber;

  int    howManyPathsAway;

  double abHeading;               //Угол линии АВ
  double angVel;
  double distanceFromRefLine;     //Расстояние от устанавливающей линии
  double distanceFromCurrentLine; //Расстояние от текущей линии
  double widthMinusOverlap;   //Ширина линий, учитывая перекрытия/пропуски
  double abFixHeadingDelta;
  double rEastAB;
  double rNorthAB;
  double ppRadiusAB;
  double steerAngleAB;

  uint8_t flags;  // Флаги
                  // 0 - Установлена точка А
                  // 1 - Установлена точка В
                  // 2 - Установлена линия АВ;     isABLineSet
                  // 3 - isABSameAsVehicleHeading;   
                  // 4 - isOnRightSideCurrentLine;                               
} ABline;

void initABl(void);

//void btnAPoint(void);


void SetABLineByHeading(void);

void MoveABLine(double dist);

void GetCurrentABLine(vec3 pivot); 

double module(double var);
double roundAwayFromZero(double var, uint8_t decimal);

/* ------------------------------------------------------- */
#endif

