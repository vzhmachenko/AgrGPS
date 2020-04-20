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

/*
  pivot.easting = 0.70;
  pivot.northing = 18.10;
  */

  //------------Расчитываем как далеко мы от первой линии АВ--------------
  //how far are we away from the reference line at 90 degrees
  abline.distanceFromRefLine  = ((dy * pivot.easting)     - (dx * pivot.northing) 
                          + (abline.refABLineP2.easting   * abline.refABLineP1.northing) 
                          - (abline.refABLineP2.northing  * abline.refABLineP1.easting))
                          / sqrt((dy * dy) + (dx * dx));

  //sign of distance determines which side of line we are on
  abline.refLineSide = abline.distanceFromRefLine > 0 ?  1 : -1;
  //absolute the distance
  abline.distanceFromRefLine  = module(abline.distanceFromRefLine);
  //Which ABLine is the vehicle on, negative is left and positive is right side
  abline.howManyPathsAway = roundAwayFromZero( abline.distanceFromRefLine / abline.widthMinusOverlap, 0);

  /*
  if(( abline.distanceFromRefLine / abline.widthMinusOverlap) < 0.0){
    if( (int8_t)( abline.distanceFromRefLine / abline.widthMinusOverlap * (-10)) < 5)
      abline.howManyPathsAway = ceil( abline.distanceFromRefLine / abline.widthMinusOverlap);
    else
      abline.howManyPathsAway = floor(abline.distanceFromRefLine / abline.widthMinusOverlap);
  }
  else{
    if( (int8_t)( abline.distanceFromRefLine / abline.widthMinusOverlap * 10) < 5)
      abline.howManyPathsAway = floor(abline.distanceFromRefLine / abline.widthMinusOverlap);
    else
      abline.howManyPathsAway = ceil( abline.distanceFromRefLine / abline.widthMinusOverlap );
  }
  */


/*
  abline.howManyPathsAway = (( abline.distanceFromRefLine / abline.widthMinusOverlap) < 0.0)
                      ?  floor(abline.distanceFromRefLine / abline.widthMinusOverlap - 0.5)
                      :  ceil( abline.distanceFromRefLine / abline.widthMinusOverlap + 0.5);
                      */

  //generate that pass number as signed integer
  abline.passNumber = (int8_t)(abline.refLineSide * abline.howManyPathsAway);
  //---------------------------------------------------------------------- 

  //calculate the new point that is number of implement widths over
  double toolOffset = vehicle.toolOffset;
  vec2 point1;

  //depending which way you are going, the offset can be either side
  point1 = (abline.flags >> isABSameAsVehicleHeading & 0x1) 
        ? (vec2){   (cos(-abline.abHeading)     * ((abline.widthMinusOverlap
                      * abline.howManyPathsAway * abline.refLineSide) 
                      - vehicle.toolOffset))    + abline.refPoint1.easting,
                    (sin(-abline.abHeading)     * ((abline.widthMinusOverlap 
                      * abline.howManyPathsAway * abline.refLineSide) 
                      - vehicle.toolOffset))    + abline.refPoint1.northing}
        : (vec2){   (cos(-abline.abHeading)     * ((abline.widthMinusOverlap
                      * abline.howManyPathsAway * abline.refLineSide) 
                      + vehicle.toolOffset))    + abline.refPoint1.easting,
                      (sin(-abline.abHeading)   * ((abline.widthMinusOverlap 
                      * abline.howManyPathsAway * abline.refLineSide) 
                      + vehicle.toolOffset))    + abline.refPoint1.northing};
  

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
  if(abline.distanceFromCurrentLine < 0){
    GPIOD->ODR |= 0x80;
  }
  else{
    GPIOD->ODR &= ~0x80;
  }

  //are we on the right side or not
  abline.flags = (abline.distanceFromCurrentLine > 0)
            ? abline.flags |   1 << isOnRightSideCurrentLine
            : abline.flags & ~(1 << isOnRightSideCurrentLine);
  
  //absolute the distance
  abline.distanceFromCurrentLine = module(abline.distanceFromCurrentLine);

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
  abline.abFixHeadingDelta = (module(position.fixHeading - abline.abHeading));

  if (abline.abFixHeadingDelta >= PI) 
      abline.abFixHeadingDelta = module(abline.abFixHeadingDelta - twoPI);

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
  abline.distanceFromCurrentLine = roundAwayFromZero(abline.distanceFromCurrentLine * 1000.0, 0);

/*
  //Convert to millimeters
  abline.distanceFromCurrentLine = (abline.distanceFromCurrentLine < 0) 
      ? floor(abline.distanceFromCurrentLine * 1000.0) 
      : ceil(abline.distanceFromCurrentLine * 1000.0);
      */

  //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
  abline.angVel = twoPI * 0.277777 * nmea.speed 
                * (tan(toRadians(abline.steerAngleAB))) / vehicle.wheelbase;

  //clamp the steering angle to not exceed safe angular velocity
  if (module(abline.angVel) > vehicle.maxAngularVelocity) {
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

double module(double var){
  if(var < 0)
    var *= -1;
  return var;
}
/* -------------------------------------------------------------- */
double 
roundAwayFromZero(double var, uint8_t decimal){
  // -0.1 0
  int round = (int)(var * pow(10, decimal + 1)) % 10;//(int)pow(10, decimal + 1);
  if(var < 0){
    if(-1 * round < 5){
      return ceil(var * pow(10, decimal)) / pow(10, decimal);
    }
    else {
      return floor(var * pow(10, decimal)) / pow(10, decimal);
    }
  }
  else {
    if(round < 5){
      return floor(var * pow(10, decimal)) / pow(10, decimal);
    }
    else {
      return ceil(var * pow(10, decimal)) / pow(10, decimal);
    }
  }
}
