#include "ABLine.h"
#include "gpio.h"

extern NMEA         pn;
extern Vehicle      vehicle;
extern position     pos;
       ABline       AB;

void 
initABl(void){
  //AB.isABLineSet = 0;
  //AB.isABSameAsVehicleHeading = 1;
  //AB.isOnRightSideCurrentLine = 1;
  AB.refLineSide = 1;
  AB.widthMinusOverlap  = vehicle.toolWidth 
                        - vehicle.toolOverlap;
  AB.refPoint1 = (vec2){0.0, 0.0};
  AB.flags = 0;
  AB.flags |= 1 << isABSameAsVehicleHeading;
  AB.flags |= 1 << isOnRightSideCurrentLine;
}

/*
 * Дейстивя при нажатии кнопки А
 */
void 
btnAPoint(void){
  // Если линия еще не задавалась, то устанавливаем точку A

  if( (AB.flags >> ABLineSet & 0x01) != 1)  {
//  if(AB.isABLineSet != 1){

    AB.refPoint1.easting = pos.pivotAxlePos.easting;
    AB.refPoint1.northing = pos.pivotAxlePos.northing;
    AB.abHeading = pos.pivotAxlePos.heading;

    AB.flags |= 1 << APointSet;    // Выставлям флаг Точки А
    LCD_Send_String(0, "A-point is Set.");
  }
  else{

    LCD_Send_String(0, "AB-Line is set already.");
    LCD_Send_String(1, "Try B to change A-point.");
  }
}

/*
 * Дейстивя при нажатии кнопки B
 */
void 
btnBPoint(void){

  //Если не установлена точка А
  if(! (AB.flags >> APointSet & 0x01 )){
    LCD_Send_String(0, "First set A-Point.");
    return;
  }

/*
  // Если расстояние между точками не достаточно для корректного 
  // определения направления линии
  //x2-x1
  double dx = AB.refABLineP2.easting - AB.refABLineP1.easting;
  //z2-z1
  double dy = AB.refABLineP2.northing - AB.refABLineP1.northing;
  double distInPoint = sqrt( (dx * dx) + (dy * dy));
	if(distInPoint < 0.00001){
    LCD_Send_String(0, "AB-Line error.");
		return;
  }
*/

  // Если линия АБ установлена( установлена точка А и В)
  // то делаем "замещение точек"
  if(AB.flags >> ABLineSet & 0x01){
		AB.refPoint1 = AB.refPoint2;
    LCD_Send_String(0, "Changed Points.");
  }
  else{
    AB.flags |= 1 << BPointSet;    // Выставлям флаг Точки B
    AB.flags |= 1 << ABLineSet;    // Выставляем флаг линии
    LCD_Send_String(0, "B-point is Set.");
  }

  AB.refPoint2.easting = pos.pivotAxlePos.easting;
  AB.refPoint2.northing = pos.pivotAxlePos.northing;

  AB.abHeading = atan2(AB.refPoint2.easting - AB.refPoint1.easting, 
                       AB.refPoint2.northing - AB.refPoint1.northing);

  if (AB.abHeading < 0)
    AB.abHeading += twoPI;

  //sin x cos z for endpoints, opposite for additional lines
  AB.refABLineP1.easting  = AB.refPoint1.easting  - (sin(AB.abHeading) * 4000.0);
  AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

  AB.refABLineP2.easting  = AB.refPoint1.easting  + (sin(AB.abHeading) * 4000.0);
  AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);
}

void 
SetABLineByHeading(void) {
  //heading is set in the AB Form
  AB.refABLineP1.easting  = AB.refPoint1.easting  - (sin(AB.abHeading) * 4000.0);
  AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

  AB.refABLineP2.easting  = AB.refPoint1.easting  + (sin(AB.abHeading) * 4000.0);
  AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);

  AB.refPoint2.easting  = AB.refABLineP2.easting;
  AB.refPoint2.northing = AB.refABLineP2.northing;

  AB.flags |= 1 << ABLineSet;
//   AB.isABLineSet = 1;
}

void 
MoveABLine(double dist) {
  double headingCalc;

  //calculate the heading 90 degrees to ref ABLine heading
  headingCalc = ( (AB.flags >> isABSameAsVehicleHeading & 0x01) != 0)  
              ? (AB.abHeading + PIBy2)
              : (AB.abHeading - PIBy2);
//    headingCalc = (AB.isABSameAsVehicleHeading != 0)  ? (AB.abHeading + PIBy2)
//                                                      : (AB.abHeading - PIBy2);

  //calculate the new points for the reference line and points
  AB.refPoint1.easting  = (sin(headingCalc) * dist) + AB.refPoint1.easting;
  AB.refPoint1.northing = (cos(headingCalc) * dist) + AB.refPoint1.northing;

  AB.refABLineP1.easting  = AB.refPoint1.easting  - (sin(AB.abHeading) * 4000.0);
  AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

  AB.refABLineP2.easting  = AB.refPoint1.easting  + (sin(AB.abHeading) * 4000.0);
  AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);

  AB.refPoint2.easting  = AB.refABLineP2.easting;
  AB.refPoint2.northing = AB.refABLineP2.northing;
}

void 
GetCurrentABLine(vec3 pivot) {
  //move the ABLine over based on the overlap amount set in vehicle
  double widthMinusOverlap = vehicle.toolWidth - vehicle.toolOverlap;

  //x2-x1
  double dx = AB.refABLineP2.easting - AB.refABLineP1.easting;
  //z2-z1
  double dy = AB.refABLineP2.northing - AB.refABLineP1.northing;
  //------------Расчитываем как далеко мы от первой линии АВ--------------
  //how far are we away from the reference line at 90 degrees
  AB.distanceFromRefLine  = ((dy * pivot.easting)     - (dx * pivot.northing) 
                          + (AB.refABLineP2.easting   * AB.refABLineP1.northing) 
                          - (AB.refABLineP2.northing  * AB.refABLineP1.easting))
                          / sqrt((dy * dy) + (dx * dx));

  //sign of distance determines which side of line we are on
  AB.refLineSide = AB.distanceFromCurrentLine > 0
                 ?  1
                 : -1;
//  if (AB.distanceFromRefLine > 0) 
//    AB.refLineSide = 1;
//  else AB.refLineSide = -1;

  //absolute the distance
  AB.distanceFromRefLine  = abs(AB.distanceFromRefLine);
  //Which ABLine is the vehicle on, negative is left and positive is right side
  AB.howManyPathsAway = (( AB.distanceFromCurrentLine / AB.widthMinusOverlap) < 0.0)
                      ?  floor(AB.distanceFromRefLine / AB.widthMinusOverlap)
                      :  ceil( AB.distanceFromRefLine / AB.widthMinusOverlap);

  //generate that pass number as signed integer
  AB.passNumber = (int8_t)(AB.refLineSide * AB.howManyPathsAway);
  //---------------------------------------------------------------------- 

  //calculate the new point that is number of implement widths over
  vec2 point1;

  //depending which way you are going, the offset can be either side
//  if (AB.alags >> 3 & 1) {
  point1 = (AB.flags >> isABSameAsVehicleHeading & 0x1) //AB.isABSameAsVehicleHeading
        ? (vec2){(cos(-AB.abHeading)     * ((AB.widthMinusOverlap
                      * AB.howManyPathsAway * AB.refLineSide) 
                      - vehicle.toolOffset))+ AB.refPoint1.easting,
                    (sin(-AB.abHeading)     * ((AB.widthMinusOverlap 
                      * AB.howManyPathsAway * AB.refLineSide) 
                      - vehicle.toolOffset))+ AB.refPoint1.northing}
        : (vec2){(cos(-AB.abHeading)   * ((AB.widthMinusOverlap
                      * AB.howManyPathsAway * AB.refLineSide) 
                      + vehicle.toolOffset))+ AB.refPoint1.easting,
                      (sin(-AB.abHeading)   * ((AB.widthMinusOverlap 
                      * AB.howManyPathsAway * AB.refLineSide) 
                      + vehicle.toolOffset))+ AB.refPoint1.northing};
  

  //create the new line extent points for current ABLine based on original heading of AB line
  AB.currentABLineP1.easting  = point1.easting  - (sin(AB.abHeading) * 40000.0);
  AB.currentABLineP1.northing = point1.northing - (cos(AB.abHeading) * 40000.0);

  AB.currentABLineP2.easting  = point1.easting  + (sin(AB.abHeading) * 40000.0);
  AB.currentABLineP2.northing = point1.northing + (cos(AB.abHeading) * 40000.0);

  //get the distance from currently active AB line
  //x2-x1
  dx = AB.currentABLineP2.easting   - AB.currentABLineP1.easting;
  //z2-z.
  dy = AB.currentABLineP2.northing  - AB.currentABLineP1.northing;

  //save a copy of dx,dy in youTurn
/*    mf.yt.dxAB = dx; mf.yt.dyAB = dy;*/

  //how far from current AB Line is fix
  AB.distanceFromCurrentLine  = ((dy * pivot.easting) - (dx * pivot.northing) 
                              + (AB.currentABLineP2.easting
                              * AB.currentABLineP1.northing) - (AB.currentABLineP2.northing 
                              * AB.currentABLineP1.easting)) / sqrt((dy * dy) + (dx * dx));

  //are we on the right side or not
//  AB.flags = AB.distanceFromCurrentLine > 0
//           ? AB.flags | 1 << 4
//           : AB.flags & !(0 << 4);
  AB.flags = (AB.distanceFromCurrentLine > 0)
            ? AB.flags |   1 << isOnRightSideCurrentLine
            : AB.flags & ~(1 << isOnRightSideCurrentLine);
//  AB.isOnRightSideCurrentLine = (AB.distanceFromCurrentLine > 0);
  
  //absolute the distance
  AB.distanceFromCurrentLine = abs(AB.distanceFromCurrentLine);
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

  //update base on autosteer settings and distance from line
  double goalPointDistance = UpdateGoalPointDistance(AB.distanceFromCurrentLine);
/*    mf.lookaheadActual = goalPointDistance;
*/
  //Subtract the two headings, if > 1.57 its going the opposite heading as refAB
  AB.abFixHeadingDelta = (abs (pos.fixHeading - AB.abHeading));

  if (AB.abFixHeadingDelta >= PI) 
      AB.abFixHeadingDelta = abs(AB.abFixHeadingDelta - twoPI);

  // ** Pure pursuit ** - calc point on ABLine closest to current position
  double U = ( ((pivot.easting  - AB.currentABLineP1.easting ) * dx)
             + ((pivot.northing - AB.currentABLineP1.northing) * dy) )
             / ((dx * dx) + (dy * dy));

  //point on AB line closest to pivot axle point
  AB.rEastAB  = AB.currentABLineP1.easting  + (U * dx);
  AB.rNorthAB = AB.currentABLineP1.northing + (U * dy);

  if (AB.abFixHeadingDelta >= PIBy2) {
      AB.flags &= ~(1 << isABSameAsVehicleHeading);
//      AB.isABSameAsVehicleHeading = 0;
      AB.goalPointAB.easting  = AB.rEastAB  - (sin(AB.abHeading) * goalPointDistance);
      AB.goalPointAB.northing = AB.rNorthAB - (cos(AB.abHeading) * goalPointDistance);
  }
  else {
      AB.flags |= 1 << isABSameAsVehicleHeading;
//      AB.isABSameAsVehicleHeading = 1;
      AB.goalPointAB.easting  = AB.rEastAB  + (sin(AB.abHeading) * goalPointDistance);
      AB.goalPointAB.northing = AB.rNorthAB + (cos(AB.abHeading) * goalPointDistance);
  }

  //calc "D" the distance from pivot axle to lookahead point
  double goalPointDistanceDSquared = DistanceSquared( AB.goalPointAB.northing, 
                                                      AB.goalPointAB.easting, 
                                                      pivot.northing, 
                                                      pivot.easting);

  //calculate the the new x in local coordinates and steering angle degrees based on wheelbase
  double localHeading = twoPI - pos.fixHeading;
  AB.ppRadiusAB = goalPointDistanceDSquared / (2 * (
                 ((AB.goalPointAB.easting  - pivot.easting)  * cos(localHeading)) 
              +  ((AB.goalPointAB.northing - pivot.northing) * sin(localHeading)) ) );

  AB.steerAngleAB = toDegrees(atan(2 * (((AB.goalPointAB.easting - pivot.easting) 
                  * cos(localHeading)) + ((AB.goalPointAB.northing - pivot.northing) 
                  * sin(localHeading))) * vehicle.wheelbase / goalPointDistanceDSquared));
  if (AB.steerAngleAB < -vehicle.maxSteerAngle) 
      AB.steerAngleAB = -vehicle.maxSteerAngle;
  if (AB.steerAngleAB > vehicle.maxSteerAngle) 
      AB.steerAngleAB = vehicle.maxSteerAngle;

  //limit circle size for display purpose
  if (AB.ppRadiusAB < -500) 
      AB.ppRadiusAB = -500;
  if (AB.ppRadiusAB > 500) 
      AB.ppRadiusAB = 500;

  AB.radiusPointAB.easting  = pivot.easting  + (AB.ppRadiusAB * cos(localHeading));
  AB.radiusPointAB.northing = pivot.northing + (AB.ppRadiusAB * sin(localHeading));

  //Convert to millimeters
  AB.distanceFromCurrentLine = (AB.distanceFromCurrentLine < 0) 
      ? floor(AB.distanceFromCurrentLine * 1000.0) 
      : ceil(AB.distanceFromCurrentLine * 1000.0);

  //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
  AB.angVel = twoPI * 0.277777 * pn.speed * (tan(toRadians(AB.steerAngleAB))) / vehicle.wheelbase;

  //clamp the steering angle to not exceed safe angular velocity
  if (abs(AB.angVel) > vehicle.maxAngularVelocity) {
    AB.steerAngleAB = toDegrees(AB.steerAngleAB > 0 
      ? (atan((vehicle.wheelbase * vehicle.maxAngularVelocity) 
              / (twoPI * pn.speed * 0.277777)))
      : (atan((vehicle.wheelbase * -vehicle.maxAngularVelocity) 
              / (twoPI * pn.speed * 0.277777))));
  }

  //distance is negative if on left, positive if on right
  AB.distanceFromCurrentLine = (AB.flags >> isABSameAsVehicleHeading & 0x01 )
                             & (AB.flags >> isOnRightSideCurrentLine & 0x01) 
                             ? AB.distanceFromCurrentLine *  1.0
                             : AB.distanceFromCurrentLine * -1.0;
//  AB.distanceFromCurrentLine = AB.isABSameAsVehicleHeading & AB.isOnRightSideCurrentLine
//                             ? AB.distanceFromCurrentLine *  1.0
//                             : AB.distanceFromCurrentLine * -1.0;
/*
  if (AB.isABSameAsVehicleHeading) {
    if (!AB.isOnRightSideCurrentLine)
      AB.distanceFromCurrentLine *= -1.0;
  }
  else {        //opposite way so right is left
    if (AB.isOnRightSideCurrentLine) 
      AB.distanceFromCurrentLine *= -1.0;
  }

*/

  pos.guidanceLineDistanceOff = (int16_t)AB.distanceFromCurrentLine;
  pos.guidanceLineSteerAngle  = (int16_t)(AB.steerAngleAB * 100);


/*    if (mf.yt.isYouTurnTriggered) {
      //do the pure pursuit from youTurn
      mf.yt.DistanceFromYouTurnLine();

      mf.seq.DoSequenceEvent();

      //now substitute what it thinks are AB line values with auto turn values
      AB.steerAngleAB = mf.yt.steerAngleYT;
      AB.distanceFromCurrentLine = mf.yt.distanceFromCurrentLine;

      AB.goalPointAB = mf.yt.goalPointYT;
      AB.radiusPointAB.easting = mf.yt.radiusPointYT.easting;
      AB.radiusPointAB.northing = mf.yt.radiusPointYT.northing;
      AB.ppRadiusAB = mf.yt.ppRadiusYT;
  }
*/    
}
