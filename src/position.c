#include "position.h"
#include "vehicle.h"
position pos;
extern NMEA pn;
extern Vehicle vehicle;

void 
initPosition(){
    pos.isFirstFixPositionSet = 0;
    pos.isGPSPositionInitialized = 0;
    pos.isFixHolding = 0;
    pos.isFixHoldLoaded = 0;
    
    pos.fixHeading = 0.0;
    pos.distance = 0;
    pos.boundaryTriggerDistance = 4;
    pos.startCounter = 0;
    pos.fixUpdateHz = 3;
    pos.totalFixSteps = 5 * 6;
    pos.distanceCurrentStepFix = 0;        
    pos.minFixStepDist = 0;
    pos.prevFix.easting = 0;
    pos.prevFix.northing = 0;
    pos.prevBoundaryPos.easting = 0;
    pos.prevBoundaryPos.northing = 0;

}

void 
CalculatePositionHeading(void){
    pos.fixHeading = toRadians(pn.headingTrue);
    //translate world to the pivot axle
    pos.pivotAxlePos.easting = pn.fix.easting - (sin(pos.fixHeading) * vehicle.antennaPivot);
    pos.pivotAxlePos.northing = pn.fix.northing - (cos(pos.fixHeading) * vehicle.antennaPivot);
    pos.pivotAxlePos.heading = pos.fixHeading;
}

void 
InitializeFirstFewGPSPositions(void){
    if (!pos.isFirstFixPositionSet) {
        //reduce the huge utm coordinates
        pn.utmEast = (int32_t)pn.fix.easting;
        pn.utmNorth = (int32_t)(pn.fix.northing);
        pn.fix.easting = pn.fix.easting - pn.utmEast;
        pn.fix.northing = pn.fix.northing - pn.utmNorth;
    
        //calculate the central meridian of current zone
        pn.centralMeridian = -177 + ((pn.zone - 1) * 6);

        //Azimuth Error - utm declination
        pn.convergenceAngle = atan(sin(toRadians(pn.latitude)) * tan(toRadians(pn.longitude - pn.centralMeridian)));
        //Вывод на екран lblConvergenceAngle.Text = Math.Round(glm.toDegrees(pn.convergenceAngle), 3).ToString();

        //Draw a grid once we know where in the world we are.
        pos.isFirstFixPositionSet = 1;

        //most recent fixes
        pos.prevFix.easting = pn.fix.easting;
        pos.prevFix.northing = pn.fix.northing;

        pos.stepFixPts[0].easting = pn.fix.easting;
        pos.stepFixPts[0].northing = pn.fix.northing;
        pos.stepFixPts[0].heading = 0;

        return;
    }
    else { 
        //most recent fixes
        pos.prevFix.easting = pn.fix.easting;
        pos.prevFix.northing = pn.fix.northing;

        //load up history with valid data
        for (int i = pos.totalFixSteps - 1; i > 0; i--) {
            pos.stepFixPts[i].easting = pos.stepFixPts[i - 1].easting;
            pos.stepFixPts[i].northing = pos.stepFixPts[i - 1].northing;
            pos.stepFixPts[i].heading = pos.stepFixPts[i - 1].heading;
        }

        pos.stepFixPts[0].heading = DistanceVec2Vec3(pn.fix, pos.stepFixPts[0]);
        pos.stepFixPts[0].easting = pn.fix.easting;
        pos.stepFixPts[0].northing = pn.fix.northing;

        //keep here till valid data
        if (pos.startCounter > (pos.totalFixSteps/2)) 
            pos.isGPSPositionInitialized = 1;

        //in radians
        pos.fixHeading = atan2(pn.fix.easting - pos.stepFixPts[pos.totalFixSteps - 1].easting, pn.fix.northing - pos.stepFixPts[pos.totalFixSteps - 1].northing);
        if (pos.fixHeading < 0) 
            pos.fixHeading += twoPI;

        //send out initial zero settings
        //if (pos.isGPSPositionInitialized) 
        //AutoSteerSettingsOutToPort();
        return;
    }
}

void 
UpdateFixPosition(void){
    pos.startCounter++;
    pos.totalFixSteps = pos.fixUpdateHz * 6;
        		GPIOD->ODR ^= 0x100;

    if(pos.isGPSPositionInitialized == 0){
        InitializeFirstFewGPSPositions();
        return;
    }
        		GPIOD->ODR ^= 0x200;

    //region Antenne Offset
    //----------------------------------------
    //----------------------------------------
    //----------------------------------------
    //REGION STEP FIX
    //grab the most current fix and save the distance from the last fix
    pos.distanceCurrentStepFix = DistanceVec2Vec3(pn.fix, pos.stepFixPts[0]);
    pos.fixStepDist = pos.distanceCurrentStepFix;

    //if  min distance isn't exceeded, keep adding old fixes till it does
    if (pos.distanceCurrentStepFix <= pos.minFixStepDist) {
        for (pos.currentStepFix = 0; pos.currentStepFix < pos.totalFixSteps; pos.currentStepFix++) {
            pos.fixStepDist += pos.stepFixPts[pos.currentStepFix].heading;
            if (pos.fixStepDist > pos.minFixStepDist) {
                //if we reached end, keep the oldest and stay till distance is exceeded
                if (pos.currentStepFix < (pos.totalFixSteps-1) ) 
                    pos.currentStepFix++;
                pos.isFixHolding = 0;
                break;
            }
            else 
                pos.isFixHolding = 1;
        }
    }

    // only takes a single fix to exceeed min distance
    else 
        pos.currentStepFix = 0;

    //if total distance is less then the addition of all the fixes, keep last one as reference
    if (pos.isFixHolding) {
        if (pos.isFixHoldLoaded== 0) {
            pos.vHold = pos.stepFixPts[(pos.totalFixSteps - 1)];
            pos.isFixHoldLoaded = 1;
        }

        //cycle thru like normal
        for (int i = pos.totalFixSteps - 1; i > 0; i--) 
            pos.stepFixPts[i] = pos.stepFixPts[i - 1];

        //fill in the latest distance and fix
        pos.stepFixPts[0].heading = DistanceVec2Vec3(pn.fix, pos.stepFixPts[0]);
        pos.stepFixPts[0].easting = pn.fix.easting;
        pos.stepFixPts[0].northing = pn.fix.northing;

        //reload the last position that was triggered.
        pos.stepFixPts[(pos.totalFixSteps - 1)].heading = DistanceVec3Vec3(pos.vHold, pos.stepFixPts[(pos.totalFixSteps - 1)]);
        pos.stepFixPts[(pos.totalFixSteps - 1)].easting = pos.vHold.easting;
        pos.stepFixPts[(pos.totalFixSteps - 1)].northing = pos.vHold.northing;
    }
    else { //distance is exceeded, time to do all calcs and next frame
        //positions and headings 
        CalculatePositionHeading();

        //get rid of hold position
        pos.isFixHoldLoaded = 0;

        //don't add the total distance again
        pos.stepFixPts[(pos.totalFixSteps - 1)].heading = 0;

        //test if travelled far enough for new boundary point
       /* double boundaryDistance = DistanceVec2Vec2(pn.fix, pos.prevBoundaryPos);
        if (boundaryDistance > pos.boundaryTriggerDistance) 
            AddBoundaryAndPerimiterPoint();*/

        //calc distance travelled since last GPS fix
        //Отключаю полевые вычисления
        //pos.distance = DistanceVec2Vec2(pn.fix, pos.prevFix);
        //if ((fd.distanceUser += pos.distance) > 3000) fd.distanceUser = 0; ;//userDistance can be reset

        //most recent fixes are now the prev ones
        pos.prevFix.easting = pn.fix.easting;
        pos.prevFix.northing = pn.fix.northing;

        //load up history with valid data
        for (int i = pos.totalFixSteps - 1; i > 0; i--) 
            pos.stepFixPts[i] = pos.stepFixPts[i - 1];
        pos.stepFixPts[0].heading = DistanceVec2Vec3(pn.fix, pos.stepFixPts[0]);
        pos.stepFixPts[0].easting = pn.fix.easting;
        pos.stepFixPts[0].northing = pn.fix.northing;
    }
}
