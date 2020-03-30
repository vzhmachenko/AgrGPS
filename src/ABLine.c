#include "ABLine.h"

#include "nmea.h"
#include "position.h"

       ABline         abline;
extern NMEA           nmea;
extern Vehicle        vehicle;
extern Position       position;
extern QueueHandle_t	lcdQueue;      //Указатель на очередь взаимодействия мужду задачами (char --> lcd)
       

void 
initABl(void){
  //abline.isABLineSet = 0;
  //abline.isABSameAsVehicleHeading = 1;
  //abline.isOnRightSideCurrentLine = 1;
  abline.refLineSide = 1;
  abline.widthMinusOverlap  = vehicle.toolWidth 
                        - vehicle.toolOverlap;
  abline.refPoint1 = (vec2){0.0, 0.0};
  abline.flags = 0;
  abline.flags |= 1 << isABSameAsVehicleHeading;
  abline.flags |= 1 << isOnRightSideCurrentLine;
}

/*
 * Дейстивя при нажатии кнопки А
 */
void 
btnAPoint(void){
  // Если линия еще не задавалась, то устанавливаем точку A

  if( !(abline.flags >> ABLineSet & 0x01) )  {

    vec3 fix = position.pivotAxlePos;

    abline.refPoint1.easting  = fix.easting;
    abline.refPoint1.northing = fix.northing;
    abline.abHeading          = fix.heading;

    abline.flags |= 1 << APointSet;    // Выставлям флаг Точки А
//    LCD_Send_String(0, "A-point is Set.");
    strToDisplay (lcdQueue, 0, "A-point is Set.");
  }
  else{
//    LCD_Send_String(0, "ABline-Line is set already.");
    strToDisplay (lcdQueue, 0, "ABline-Line is set already.");
//    LCD_Send_String(1, "Try B to change A-point.");
    strToDisplay (lcdQueue, 1, "Try B to change A-point.");
  }
}

/*
 * Дейстивя при нажатии кнопки B
 */
void 
btnBPoint(void){
  //Если не установлена точка А
  if(! (abline.flags >> APointSet & 0x01 )){
    //LCD_Send_String(0, "First set A-Point.");
    strToDisplay (lcdQueue, 0, "First set A-Point.");
    return;
  }
//<commentingTAG>
/*
  // Если расстояние между точками не достаточно для корректного 
  // определения направления линии
  //x2-x1
  double dx = abline.refABLineP2.easting - abline.refABLineP1.easting;
  //z2-z1
  double dy = abline.refABLineP2.northing - abline.refABLineP1.northing;
  double distInPoint = sqrt( (dx * dx) + (dy * dy));
	if(distInPoint < 0.00001){
    LCD_Send_String(0, "abline-Line error.");
		return;
  }
*/ 
//</commentingTAG>

  // Если линия АБ установлена( установлена точка А и В)
  // то делаем "замещение точек"
  if(abline.flags >> ABLineSet & 0x01){
		abline.refPoint1 = abline.refPoint2;
    lineParam t;
    initLCDstruct (&t, 0, "Changed Points.");
    xQueueSendToBack(lcdQueue, &t, 50);
  }

  abline.refPoint2.easting  = nmea.fix.easting;
  abline.refPoint2.northing = nmea.fix.northing;

  abline.abHeading = atan2(abline.refPoint2.easting  - abline.refPoint1.easting, 
                           abline.refPoint2.northing - abline.refPoint1.northing);
  if (abline.abHeading < 0)
    abline.abHeading += twoPI;

  //sin x cos z for endpoints, opposite for additional lines
  abline.refABLineP1.easting  = abline.refPoint1.easting  - (sin(abline.abHeading) * 1600.0);
  abline.refABLineP1.northing = abline.refPoint1.northing - (cos(abline.abHeading) * 1600.0);

  abline.refABLineP2.easting  = abline.refPoint1.easting  + (sin(abline.abHeading) * 1600.0);
  abline.refABLineP2.northing = abline.refPoint1.northing + (cos(abline.abHeading) * 1600.0);

  abline.flags |= 1 << BPointSet;    // Выставлям флаг Точки B
  abline.flags |= 1 << ABLineSet;    // Выставляем флаг линии

  lineParam t;
  initLCDstruct (&t, 0, "B-point is Set.");
  xQueueSendToBack(lcdQueue, &t, 50);
}

void 
SetABLineByHeading(void) {
  //heading is set in the ablineForm
  abline.refABLineP1.easting  = abline.refPoint1.easting  - (sin(abline.abHeading) * 1600.0);
  abline.refABLineP1.northing = abline.refPoint1.northing - (cos(abline.abHeading) * 1600.0);

  abline.refABLineP2.easting  = abline.refPoint1.easting  + (sin(abline.abHeading) * 1600.0);
  abline.refABLineP2.northing = abline.refPoint1.northing + (cos(abline.abHeading) * 1600.0);

  abline.refPoint2.easting  = abline.refABLineP2.easting;
  abline.refPoint2.northing = abline.refABLineP2.northing;

  abline.flags |= 1 << ABLineSet;
//   abline.isABLineSet = 1;
}

void 
MoveABLine(double dist) {
  double headingCalc;

  //calculate the heading 90 degrees to ref ABLine heading
  headingCalc = ( (abline.flags >> isABSameAsVehicleHeading & 0x01) != 0)  
              ? (abline.abHeading + PIBy2)
              : (abline.abHeading - PIBy2);
//    headingCalc = (abline.isABSameAsVehicleHeading != 0)  ? (abline.abHeading + PIBy2)
//                                                      : (abline.abHeading - PIBy2);

  //calculate the new points for the reference line and points
  abline.refPoint1.easting  = (sin(headingCalc) * dist) + abline.refPoint1.easting;
  abline.refPoint1.northing = (cos(headingCalc) * dist) + abline.refPoint1.northing;

  abline.refABLineP1.easting  = abline.refPoint1.easting  - (sin(abline.abHeading) * 4000.0);
  abline.refABLineP1.northing = abline.refPoint1.northing - (cos(abline.abHeading) * 4000.0);

  abline.refABLineP2.easting  = abline.refPoint1.easting  + (sin(abline.abHeading) * 4000.0);
  abline.refABLineP2.northing = abline.refPoint1.northing + (cos(abline.abHeading) * 4000.0);

  abline.refPoint2.easting  = abline.refABLineP2.easting;
  abline.refPoint2.northing = abline.refABLineP2.northing;
}

void 
GetCurrentABLine(vec3 pivot) {
  //move the ABLine over based on the overlap amount set in vehicle
  double widthMinusOverlap = vehicle.toolWidth - vehicle.toolOverlap;

  //x2-x1
  double dx = abline.refABLineP2.easting - abline.refABLineP1.easting;
  //z2-z1
  double dy = abline.refABLineP2.northing - abline.refABLineP1.northing;
  //------------Расчитываем как далеко мы от первой линии АВ--------------
  //how far are we away from the reference line at 90 degrees
  abline.distanceFromRefLine  = ((dy * pivot.easting)     - (dx * pivot.northing) 
                          + (abline.refABLineP2.easting   * abline.refABLineP1.northing) 
                          - (abline.refABLineP2.northing  * abline.refABLineP1.easting))
                          / sqrt((dy * dy) + (dx * dx));

  //sign of distance determines which side of line we are on
  abline.refLineSide = abline.distanceFromCurrentLine > 0
                     ?  1
                     : -1;
  //absolute the distance
  abline.distanceFromRefLine  = abs(abline.distanceFromRefLine);
  //Which ABLine is the vehicle on, negative is left and positive is right side
  abline.howManyPathsAway = (( abline.distanceFromCurrentLine / abline.widthMinusOverlap) < 0.0)
                      ?  floor(abline.distanceFromRefLine / abline.widthMinusOverlap)
                      :  ceil( abline.distanceFromRefLine / abline.widthMinusOverlap);

  //generate that pass number as signed integer
  abline.passNumber = (int8_t)(abline.refLineSide * abline.howManyPathsAway);
  //---------------------------------------------------------------------- 

  //calculate the new point that is number of implement widths over
  double toolOffset = vehicle.toolOffset;
  vec2 point1;

  //depending which way you are going, the offset can be either side
  point1 = (abline.flags >> isABSameAsVehicleHeading & 0x1) 
        ? (vec2){(cos(-abline.abHeading)     * ((abline.widthMinusOverlap
                      * abline.howManyPathsAway * abline.refLineSide) 
                      - vehicle.toolOffset))+ abline.refPoint1.easting,
                    (sin(-abline.abHeading)     * ((abline.widthMinusOverlap 
                      * abline.howManyPathsAway * abline.refLineSide) 
                      - vehicle.toolOffset))+ abline.refPoint1.northing}
        : (vec2){(cos(-abline.abHeading)   * ((abline.widthMinusOverlap
                      * abline.howManyPathsAway * abline.refLineSide) 
                      + vehicle.toolOffset))+ abline.refPoint1.easting,
                      (sin(-abline.abHeading)   * ((abline.widthMinusOverlap 
                      * abline.howManyPathsAway * abline.refLineSide) 
                      + vehicle.toolOffset))+ abline.refPoint1.northing};
  

  //create the new line extent points for current ABLine based on original heading of ablineline
  abline.currentABLineP1.easting  = point1.easting  - (sin(abline.abHeading) * 1600.0);
  abline.currentABLineP1.northing = point1.northing - (cos(abline.abHeading) * 1600.0);

  abline.currentABLineP2.easting  = point1.easting  + (sin(abline.abHeading) * 1600.0);
  abline.currentABLineP2.northing = point1.northing + (cos(abline.abHeading) * 1600.0);

  //get the distance from currently active ablineline
  //x2-x1
  dx = abline.currentABLineP2.easting   - abline.currentABLineP1.easting;
  //z2-z.
  dy = abline.currentABLineP2.northing  - abline.currentABLineP1.northing;

  //how far from current ablineLine is fix
  abline.distanceFromCurrentLine  = ((dy * pivot.easting) - (dx * pivot.northing) 
                              + (abline.currentABLineP2.easting
                              * abline.currentABLineP1.northing) - (abline.currentABLineP2.northing 
                              * abline.currentABLineP1.easting)) / sqrt((dy * dy) + (dx * dx));

  //are we on the right side or not
  abline.flags = (abline.distanceFromCurrentLine > 0)
            ? abline.flags |   1 << isOnRightSideCurrentLine
            : abline.flags & ~(1 << isOnRightSideCurrentLine);
  
  //absolute the distance
  abline.distanceFromCurrentLine = abs(abline.distanceFromCurrentLine);
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

  //update base on autosteer settings and distance from line
  double goalPointDistance = UpdateGoalPointDistance(abline.distanceFromCurrentLine);
// <ComentingTag>  
/*    mf.lookaheadActual = goalPointDistance;
*/
// </ComentingTag>  
  //Subtract the two headings, if > 1.57 its going the opposite heading as refAB
  abline.abFixHeadingDelta = (abs (position.fixHeading - abline.abHeading));

  if (abline.abFixHeadingDelta >= PI) 
      abline.abFixHeadingDelta = abs(abline.abFixHeadingDelta - twoPI);

  // ** Pure pursuit ** - calc point on ABLine closest to current position
  double U = ( ((pivot.easting  - abline.currentABLineP1.easting ) * dx)
             + ((pivot.northing - abline.currentABLineP1.northing) * dy) )
             / ((dx * dx) + (dy * dy));

  //point on ablineline closest to pivot axle point
  abline.rEastAB  = abline.currentABLineP1.easting  + (U * dx);
  abline.rNorthAB = abline.currentABLineP1.northing + (U * dy);

  if (abline.abFixHeadingDelta >= PIBy2) {
      abline.flags &= ~(0x01 << isABSameAsVehicleHeading);
      abline.goalPointAB.easting  = abline.rEastAB  - (sin(abline.abHeading) * goalPointDistance);
      abline.goalPointAB.northing = abline.rNorthAB - (cos(abline.abHeading) * goalPointDistance);
  }
  else {
      abline.flags |= 0x01 << isABSameAsVehicleHeading;
      abline.goalPointAB.easting  = abline.rEastAB  + (sin(abline.abHeading) * goalPointDistance);
      abline.goalPointAB.northing = abline.rNorthAB + (cos(abline.abHeading) * goalPointDistance);
  }

  //calc "D" the distance from pivot axle to lookahead point
  double goalPointDistanceDSquared = DistanceSquared( abline.goalPointAB.northing, 
                                                      abline.goalPointAB.easting, 
                                                      pivot.northing, 
                                                      pivot.easting);

  //calculate the the new x in local coordinates and steering angle degrees based on wheelbase
  double localHeading = twoPI - position.fixHeading;
  abline.ppRadiusAB = goalPointDistanceDSquared / (2 * (
                 ((abline.goalPointAB.easting  - pivot.easting)  * cos(localHeading)) 
              +  ((abline.goalPointAB.northing - pivot.northing) * sin(localHeading)) ) );

  abline.steerAngleAB = toDegrees(atan(2 * (((abline.goalPointAB.easting - pivot.easting) 
                  * cos(localHeading)) + ((abline.goalPointAB.northing - pivot.northing) 
                  * sin(localHeading))) * vehicle.wheelbase / goalPointDistanceDSquared));
  if (abline.steerAngleAB < -vehicle.maxSteerAngle) 
      abline.steerAngleAB = -vehicle.maxSteerAngle;
  if (abline.steerAngleAB > vehicle.maxSteerAngle) 
      abline.steerAngleAB = vehicle.maxSteerAngle;

  //limit circle size for display purpose
  if (abline.ppRadiusAB < -500) 
      abline.ppRadiusAB = -500;
  if (abline.ppRadiusAB > 500) 
      abline.ppRadiusAB = 500;

  abline.radiusPointAB.easting  = pivot.easting  + (abline.ppRadiusAB * cos(localHeading));
  abline.radiusPointAB.northing = pivot.northing + (abline.ppRadiusAB * sin(localHeading));

  //Convert to millimeters
  abline.distanceFromCurrentLine = (abline.distanceFromCurrentLine < 0) 
      ? floor(abline.distanceFromCurrentLine * 1000.0) 
      : ceil(abline.distanceFromCurrentLine * 1000.0);

  //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
  abline.angVel = twoPI * 0.277777 * nmea.speed 
                * (tan(toRadians(abline.steerAngleAB))) / vehicle.wheelbase;

  //clamp the steering angle to not exceed safe angular velocity
  if (abs(abline.angVel) > vehicle.maxAngularVelocity) {
    abline.steerAngleAB = toDegrees(abline.steerAngleAB > 0 
      ? (atan((vehicle.wheelbase * vehicle.maxAngularVelocity) 
              / (twoPI * nmea.speed * 0.277777)))
      : (atan((vehicle.wheelbase * -vehicle.maxAngularVelocity) 
              / (twoPI * nmea.speed * 0.277777))));
  }

  //distance is negative if on left, positive if on right
  if (abline.flags >> isABSameAsVehicleHeading & 0x01) {
    if (!abline.flags >> isOnRightSideCurrentLine & 0x01){
      abline.distanceFromCurrentLine *= -1.0;
    }
  }
  else {        //opposite way so right is left
    if (abline.flags >> isOnRightSideCurrentLine & 0x01) {
      abline.distanceFromCurrentLine *= -1.0;
    }
  }

  position.guidanceLineDistanceOff = (int16_t)abline.distanceFromCurrentLine;
  position.guidanceLineSteerAngle  = (int16_t)(abline.steerAngleAB * 100);
}

/* -------------------------------------------------------------- */