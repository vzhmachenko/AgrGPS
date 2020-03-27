#include "vehicle.h"

#include "nmea.h"
        Vehicle vehicle;
extern  NMEA nmea;

double 
UpdateGoalPointDistance(double distanceFromCurrentLine) {
  //how far should goal point be away  - speed * seconds * kmph -> m/s then limit min value

  double goalPointDistance  = nmea.speed 
                            * vehicle.goalPointLookAheadSeconds 
                            * 0.27777777;//vehicle.goalPointLookAheadSeconds

  goalPointDistance = (distanceFromCurrentLine < 1.0)
    ?  goalPointDistance  + distanceFromCurrentLine
                          *  goalPointDistance 
                          *  vehicle.goalPointDistanceMultiplier
    :  goalPointDistance  + goalPointDistance
                          *  vehicle.goalPointDistanceMultiplier;

  if (goalPointDistance < vehicle.goalPointLookAheadMinimumDistance) 
      goalPointDistance = vehicle.goalPointLookAheadMinimumDistance;

  //mf.lookaheadActual = goalPointDistance;

  return goalPointDistance;
}

void 
initVehicle(void){
  vehicle.toolWidth     = 4;
  vehicle.toolOverlap   = 0;
  vehicle.toolOffset    = 0;

  vehicle.antennaHeight = 1;
  vehicle.antennaOffset = 0;
  vehicle.antennaPivot  = 0.1;
  vehicle.maxSteerAngle = 40;
  vehicle.wheelbase     = 4;
  vehicle.hitchLength   = -0.5;

  vehicle.minTurningRadius    = 6;
  vehicle.maxAngularVelocity  = 7;

  vehicle.goalPointLookAheadSeconds         = 3.0;
  vehicle.goalPointDistanceMultiplier       = 0.1;
  vehicle.goalPointLookAheadMinimumDistance = 2.0;
}
