#ifndef _ABLine_H_
#define _ABLine_H_ 

#include "vehicle.h"
#include "position.h"
#include "glm.h"
#include "nmea.h"
#include <math.h>

typedef struct {
    int isABLineSet;                //bool Установлена линия?
    int isABSameAsVehicleHeading;   //bool
    int isOnRightSideCurrentLine;
    vec2 refPoint1;                 //Установленная точка А
    vec2 refPoint2;                 //Установленная точка В
    vec2 refABLineP1;               //Пересчитан конец точки А
    vec2 refABLineP2;               //Пересчитан конец точки В
    vec2 currentABLineP1;           //Точка А линии, по которой движемся
    vec2 currentABLineP2;           //Точка B линии, по которой движемся
    vec2 goalPointAB;
    vec2 radiusPointAB;

    double abHeading;               //Угол линии АВ
    double angVel;
    double distanceFromRefLine;     //Расстояние от устанавливающей линии
    double distanceFromCurrentLine; //Расстояние от текущей линии
    double refLineSide;
    int howManyPathsAway;
    double widthMinusOverlap;   //Ширина линий, учитывая перекрытия/пропуски
    double passNumber;
    double abFixHeadingDelta;
    double rEastAB;
    double rNorthAB;
    double ppRadiusAB;
    double steerAngleAB;
} ABline;
void btnAPoint(void);
void btnBPoint(void);
void SetABLineByHeading(void);
void MoveABLine(double dist);
void initABl(void);
#endif

