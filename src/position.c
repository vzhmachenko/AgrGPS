#include "position.h"

#include "nmea.h"

       Position position;
extern NMEA     nmea;
extern Vehicle  vehicle;


/*!
 Начальные значения параметров
*/
void 
initPosition(){
  position.flags                     = 0x00;
  position.flags |= (0x01 <<  isFixHolding);      /// Пока так

  position.isGPSPositionInitialized  = 0;
  
  position.fixHeading                = 0.0;
  position.distance                  = 0;
  position.boundaryTriggerDistance   = 4;
  position.distanceCurrentStepFix    = 0;        
  position.minFixStepDist            = 10;
  position.prevFix.easting           = 0;
  position.prevFix.northing          = 0;
  position.prevBoundaryPos.easting   = 0;
  position.prevBoundaryPos.northing  = 0;
  position.offset = 0;

//  memset(position.stepFixPts, 0, sizeof(position.stepFixPts) * histSize );

}

/*
  Расчет направления (в радианах)
*/
void 
CalculatePositionHeading(void){
  position.fixHeading = toRadians(nmea.headingTrue);

  //translate world to the pivot axle
  position.pivotAxlePos.easting  = nmea.fix.easting  - (sin(position.fixHeading) 
                                 * vehicle.antennaPivot);
  position.pivotAxlePos.northing = nmea.fix.northing - (cos(position.fixHeading) 
                                 * vehicle.antennaPivot);
  position.pivotAxlePos.heading  = position.fixHeading;
}

/*!
  Уточняем начальные точки
*/
void 
InitializeFirstFewGPSPositions(void){
  static uint8_t histChange = 0;
          histChange %= 6;//10

  if ( !(position.flags << isFirstFixPositionSet & 0x01)) {

    // Уменьшаем большие значения UTM координат
    nmea.utmEast      = (int32_t)(nmea.fix.easting);
    nmea.utmNorth     = (int32_t)(nmea.fix.northing);
    nmea.fix.easting  = nmea.fix.easting - nmea.utmEast;
    nmea.fix.northing = nmea.fix.northing - nmea.utmNorth;

    // Расчет центральног меридиана текущей зоны
    nmea.centralMeridian = -177 + ((nmea.zone - 1) * 6);

    //Azimuth Error - utm declination
    nmea.convergenceAngle = atan( sin( toRadians(nmea.latitude) ) 
                          * tan( toRadians( nmea.longitude - nmea.centralMeridian) ) );

    //Draw a grid once we know where in the world we are.
    position.stepFixPts[0].heading   = 0;
    position.stepFixPts[0].northing  = nmea.fix.northing;
    position.stepFixPts[0].easting   = nmea.fix.easting;

    position.flags |= 0x01 << isFirstFixPositionSet;

    return;
  }
  else { 
    position.prevFix.easting  = nmea.fix.easting;
    position.prevFix.northing = nmea.fix.northing;
    // Пишем в историю каждое десятое измерение
    // Типа большая дискретизация
    for (int i = histSize - 1; i > 0; i--) {
      position.stepFixPts[i].easting  = position.stepFixPts[i - 1].easting;
      position.stepFixPts[i].northing = position.stepFixPts[i - 1].northing;
      position.stepFixPts[i].heading  = position.stepFixPts[i - 1].heading;
    }

    position.stepFixPts[0].heading  = DistanceVec2Vec3(nmea.fix, position.stepFixPts[0]);
    position.stepFixPts[0].easting  = nmea.fix.easting;
    position.stepFixPts[0].northing = nmea.fix.northing;

    // Если заполнили весь журнал коордитанами, то считаем,
    // что gps инициализирован
    if( histChange > histSize/2){
      position.flags |= 0x01 << isGPSPositionInitialized;
    }

    //in radians
    position.fixHeading = atan2(nmea.fix.easting  - position.stepFixPts[histSize - 1].easting,
                                nmea.fix.northing - position.stepFixPts[histSize - 1].northing);
    if (position.fixHeading < 0) 
      position.fixHeading += twoPI;
  }

  histChange++;
}

/*
  Обновляем фиксированую позицию
*/
void 
UpdateFixPosition(void){
  if( !(position.flags >> isGPSPositionInitialized & 0x01)){
    InitializeFirstFewGPSPositions();
    return;
  }

//--------------------------------------------------------------------------//
//                   region Antenne Offset
/*
  if(vehicle.antennaOffset != 0){
    position.offset = vehicle.antennaOffset;
    nmea.fix.easting  = cos(-position.fixHeading) * position.offset + nmea.fix.easting;
    nmea.fix.northing = sin(-position.fixHeading) * position.offset + nmea.fix.northing;
  }
//--------------------------------------------------------------------------// */



/*
//--------------------------------------------------------------------------//
//                          REGION STEP FIX
//--------------------------------------------------------------------------//
  //grab the most current fix and save the distance from the last fix
  // <comentingTag>
  position.distanceCurrentStepFix  = DistanceVec2Vec3(nmea.fix, position.stepFixPts[0]);
  position.fixStepDist             = position.distanceCurrentStepFix;

  //if  min distance isn't exceeded, keep adding old fixes till it does
  if (position.distanceCurrentStepFix <= position.minFixStepDist) {
    for ( position.currentStepFix = 0; 
          position.currentStepFix < position.totalFixSteps; 
          ++position.currentStepFix) {
      position.fixStepDist += position.stepFixPts[position.currentStepFix].heading;
      if (position.fixStepDist > position.minFixStepDist) {
        //if we reached end, keep the oldest and stay till distance is exceeded
        if (position.currentStepFix < (position.totalFixSteps-1) ) 
          position.currentStepFix++;
        position.isFixHolding = 0;
        break;
      }
      else 
        position.isFixHolding = 1;
    }
  }
  
/*
  // only takes a single fix to exceeed min distance
  else 
      position.currentStepFix = 0;
      */
  // </comentingTag>

  //if total distance is less then the addition of all the fixes, keep last one as reference
  if (position.flags >> isFixHolding & 0x01) {
    if ( !(position.flags >> isFixHoldLoaded & 0x01)) {
      position.vHold           = position.stepFixPts[histSize - 1];
      position.flags |= (0x01 << isFixHoldLoaded);
    }

    //cycle thru like normal
    for (int i = histSize - 1; i > 0; --i){
      position.stepFixPts[i] = position.stepFixPts[i - 1];
    }

    //fill in the latest distance and fix
    position.stepFixPts[0].heading   = DistanceVec2Vec3(nmea.fix, position.stepFixPts[0]);
    position.stepFixPts[0].easting   = nmea.fix.easting;
    position.stepFixPts[0].northing  = nmea.fix.northing;

    //reload the last position that was triggered.
    position.stepFixPts[(histSize - 1)].heading   = 
                                  DistanceVec3Vec3( position.vHold, 
                                  position.stepFixPts[(histSize - 1)]);
    position.stepFixPts[(histSize - 1)].easting   = position.vHold.easting;
    position.stepFixPts[(histSize - 1)].northing  = position.vHold.northing;
  }
  else { //distance is exceeded, time to do all calcs and next frame
    //get rid of hold position
    position.flags &= ~(0x01 << isFixHoldLoaded);

    //don't add the total distance again
    position.stepFixPts[( histSize - 1)].heading = 0;

    //positions and headings 
    CalculatePositionHeading();

    //Отключаю полевые вычисления
    position.distance = DistanceVec2Vec2(nmea.fix, position.prevFix);

    //most recent fixes are now the prev ones
    position.prevFix.easting   = nmea.fix.easting;
    position.prevFix.northing  = nmea.fix.northing;

    //load up history with valid data
    for (int i = histSize - 1; i > 0; --i){
      position.stepFixPts[i] = position.stepFixPts[i - 1];
    }

    position.stepFixPts[0].heading   = DistanceVec2Vec3(nmea.fix, position.stepFixPts[0]);
    position.stepFixPts[0].easting   = nmea.fix.easting;
    position.stepFixPts[0].northing  = nmea.fix.northing;
  }
}

/* ----------------------------------------------------------- */