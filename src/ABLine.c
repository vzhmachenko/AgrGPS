#include "ABLine.h"

extern NMEA pn;
extern Vehicle vehicle;
ABline AB;




void initABl(void){
    AB.isABLineSet = 0;
    AB.isABSameAsVehicleHeading = 1;
    AB.refLineSide = 1.0;
    AB.widthMinusOverlap = vehicle.toolWidth - vehicle.toolOverlap;
}

void btnAPoint(void){
    AB.refPoint1.easting = pn.fix.easting;
    AB.refPoint1.northing = pn.fix.northing;
}
void btnBPoint(void){
    AB.refPoint2 = (vec2){pn.fix.easting, pn.fix.northing};
    
    AB.abHeading = atan2(AB.refPoint2.easting - AB.refPoint1.easting, 
                        AB.refPoint2.northing - AB.refPoint1.northing);
    if (AB.abHeading < 0)
         AB.abHeading += twoPI;

    //sin x cos z for endpoints, opposite for additional lines
    AB.refABLineP1.easting = AB.refPoint1.easting - (sin(AB.abHeading) * 4000.0);
    AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

    AB.refABLineP2.easting = AB.refPoint1.easting + (sin(AB.abHeading) * 4000.0);
    AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);

    AB.isABLineSet = 1;
}
void SetABLineByHeading(void) {
    //heading is set in the AB Form
    AB.refABLineP1.easting = AB.refPoint1.easting - (sin(AB.abHeading) * 4000.0);
    AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

    AB.refABLineP2.easting = AB.refPoint1.easting + (sin(AB.abHeading) * 4000.0);
    AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);

    AB.refPoint2.easting = AB.refABLineP2.easting;
    AB.refPoint2.northing = AB.refABLineP2.northing;

    AB.isABLineSet = 1;
}

void MoveABLine(double dist) {
    double headingCalc;

    //calculate the heading 90 degrees to ref ABLine heading
    headingCalc = (AB.isABSameAsVehicleHeading != 0) ? (AB.abHeading + PIBy2):(AB.abHeading - PIBy2);

    //calculate the new points for the reference line and points
    AB.refPoint1.easting = (sin(headingCalc) * dist) + AB.refPoint1.easting;
    AB.refPoint1.northing = (cos(headingCalc) * dist) + AB.refPoint1.northing;

    AB.refABLineP1.easting = AB.refPoint1.easting - (sin(AB.abHeading) * 4000.0);
    AB.refABLineP1.northing = AB.refPoint1.northing - (cos(AB.abHeading) * 4000.0);

    AB.refABLineP2.easting = AB.refPoint1.easting + (sin(AB.abHeading) * 4000.0);
    AB.refABLineP2.northing = AB.refPoint1.northing + (cos(AB.abHeading) * 4000.0);

    AB.refPoint2.easting = AB.refABLineP2.easting;
    AB.refPoint2.northing = AB.refABLineP2.northing;
}
void GetCurrentABLine(vec3 pivot) {
    //move the ABLine over based on the overlap amount set in vehicle
    double widthMinusOverlap = vehicle.toolWidth - vehicle.toolOverlap;

    //x2-x1
    double dx = AB.refABLineP2.easting - AB.refABLineP1.easting;
    //z2-z1
    double dy = AB.refABLineP2.northing - AB.refABLineP1.northing;

    //how far are we away from the reference line at 90 degrees
    AB.distanceFromRefLine = ((dy * pivot.easting) - (dx * pivot.northing) + (AB.refABLineP2.easting
                            * AB.refABLineP1.northing) - (AB.refABLineP2.northing * AB.refABLineP1.easting))
                                / sqrt((dy * dy) + (dx * dx));

    //sign of distance determines which side of line we are on
    if (AB.distanceFromRefLine > 0) 
        AB.refLineSide = 1;
    else AB.refLineSide = -1;

    //absolute the distance
    AB.distanceFromRefLine = abs(AB.distanceFromRefLine);

    //Which ABLine is the vehicle on, negative is left and positive is right side
    if(AB.distanceFromCurrentLine < 0 || AB.widthMinusOverlap < 0)
        AB.howManyPathsAway = floor(AB.distanceFromRefLine / AB.widthMinusOverlap);
    else
        AB.howManyPathsAway = cerl(AB.distanceFromRefLine / AB.widthMinusOverlap);
    
/*
    //generate that pass number as signed integer
    AB.passNumber = (int)(AB.refLineSide * AB.howManyPathsAway);

    //calculate the new point that is number of implement widths over
    double toolOffset = vehicle.toolOffset;
    vec2 point1;

    //depending which way you are going, the offset can be either side
    if (AB.isABSameAsVehicleHeading)
    {
        point1 = new vec2((Math.Cos(-abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide) - toolOffset)) + refPoint1.easting,
        (Math.Sin(-abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide) - toolOffset)) + refPoint1.northing);
    }
    else
    {
        point1 = new vec2((Math.Cos(-abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide) + toolOffset)) + refPoint1.easting,
            (Math.Sin(-abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide) + toolOffset)) + refPoint1.northing);
    }

    //create the new line extent points for current ABLine based on original heading of AB line
    currentABLineP1.easting = point1.easting - (Math.Sin(abHeading) * 40000.0);
    currentABLineP1.northing = point1.northing - (Math.Cos(abHeading) * 40000.0);

    currentABLineP2.easting = point1.easting + (Math.Sin(abHeading) * 40000.0);
    currentABLineP2.northing = point1.northing + (Math.Cos(abHeading) * 40000.0);

    //get the distance from currently active AB line
    //x2-x1
    dx = currentABLineP2.easting - currentABLineP1.easting;
    //z2-z1
    dy = currentABLineP2.northing - currentABLineP1.northing;

    //save a copy of dx,dy in youTurn
    mf.yt.dxAB = dx; mf.yt.dyAB = dy;

    //how far from current AB Line is fix
    distanceFromCurrentLine = ((dy * pivot.easting) - (dx * pivot.northing) + (currentABLineP2.easting
                * currentABLineP1.northing) - (currentABLineP2.northing * currentABLineP1.easting))
                / Math.Sqrt((dy * dy) + (dx * dx));

    //are we on the right side or not
    isOnRightSideCurrentLine = distanceFromCurrentLine > 0;

    //absolute the distance
    distanceFromCurrentLine = Math.Abs(distanceFromCurrentLine);

    //update base on autosteer settings and distance from line
    double goalPointDistance = mf.vehicle.UpdateGoalPointDistance(distanceFromCurrentLine);
    mf.lookaheadActual = goalPointDistance;

    //Subtract the two headings, if > 1.57 its going the opposite heading as refAB
    abFixHeadingDelta = (Math.Abs(mf.fixHeading - abHeading));
    if (abFixHeadingDelta >= Math.PI) abFixHeadingDelta = Math.Abs(abFixHeadingDelta - glm.twoPI);

    // ** Pure pursuit ** - calc point on ABLine closest to current position
    double U = (((pivot.easting - currentABLineP1.easting) * dx)
                + ((pivot.northing - currentABLineP1.northing) * dy))
                / ((dx * dx) + (dy * dy));

    //point on AB line closest to pivot axle point
    rEastAB = currentABLineP1.easting + (U * dx);
    rNorthAB = currentABLineP1.northing + (U * dy);

    if (abFixHeadingDelta >= glm.PIBy2)
    {
        isABSameAsVehicleHeading = false;
        goalPointAB.easting = rEastAB - (Math.Sin(abHeading) * goalPointDistance);
        goalPointAB.northing = rNorthAB - (Math.Cos(abHeading) * goalPointDistance);
    }
    else
    {
        isABSameAsVehicleHeading = true;
        goalPointAB.easting = rEastAB + (Math.Sin(abHeading) * goalPointDistance);
        goalPointAB.northing = rNorthAB + (Math.Cos(abHeading) * goalPointDistance);
    }

    //calc "D" the distance from pivot axle to lookahead point
    double goalPointDistanceDSquared
        = glm.DistanceSquared(goalPointAB.northing, goalPointAB.easting, pivot.northing, pivot.easting);

    //calculate the the new x in local coordinates and steering angle degrees based on wheelbase
    double localHeading = glm.twoPI - mf.fixHeading;
    ppRadiusAB = goalPointDistanceDSquared / (2 * (((goalPointAB.easting - pivot.easting) * Math.Cos(localHeading))
        + ((goalPointAB.northing - pivot.northing) * Math.Sin(localHeading))));

    steerAngleAB = glm.toDegrees(Math.Atan(2 * (((goalPointAB.easting - pivot.easting) * Math.Cos(localHeading))
        + ((goalPointAB.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase
        / goalPointDistanceDSquared));
    if (steerAngleAB < -mf.vehicle.maxSteerAngle) steerAngleAB = -mf.vehicle.maxSteerAngle;
    if (steerAngleAB > mf.vehicle.maxSteerAngle) steerAngleAB = mf.vehicle.maxSteerAngle;

    //limit circle size for display purpose
    if (ppRadiusAB < -500) ppRadiusAB = -500;
    if (ppRadiusAB > 500) ppRadiusAB = 500;

    radiusPointAB.easting = pivot.easting + (ppRadiusAB * Math.Cos(localHeading));
    radiusPointAB.northing = pivot.northing + (ppRadiusAB * Math.Sin(localHeading));

    //Convert to millimeters
    distanceFromCurrentLine = Math.Round(distanceFromCurrentLine * 1000.0, MidpointRounding.AwayFromZero);

    //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
    angVel = glm.twoPI * 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(steerAngleAB))) / mf.vehicle.wheelbase;

    //clamp the steering angle to not exceed safe angular velocity
    if (Math.Abs(angVel) > mf.vehicle.maxAngularVelocity)
    {
        steerAngleAB = glm.toDegrees(steerAngleAB > 0 ? (Math.Atan((mf.vehicle.wheelbase * mf.vehicle.maxAngularVelocity)
            / (glm.twoPI * mf.pn.speed * 0.277777)))
            : (Math.Atan((mf.vehicle.wheelbase * -mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.pn.speed * 0.277777))));
    }

    //distance is negative if on left, positive if on right
    if (isABSameAsVehicleHeading)
    {
        if (!isOnRightSideCurrentLine) distanceFromCurrentLine *= -1.0;
    }

    //opposite way so right is left
    else
    {
        if (isOnRightSideCurrentLine) distanceFromCurrentLine *= -1.0;
    }

    mf.guidanceLineDistanceOff = (Int16)distanceFromCurrentLine;
    mf.guidanceLineSteerAngle = (Int16)(steerAngleAB * 100);

    if (mf.yt.isYouTurnTriggered)
    {
        //do the pure pursuit from youTurn
        mf.yt.DistanceFromYouTurnLine();

        mf.seq.DoSequenceEvent();

        //now substitute what it thinks are AB line values with auto turn values
        steerAngleAB = mf.yt.steerAngleYT;
        distanceFromCurrentLine = mf.yt.distanceFromCurrentLine;

        goalPointAB = mf.yt.goalPointYT;
        radiusPointAB.easting = mf.yt.radiusPointYT.easting;
        radiusPointAB.northing = mf.yt.radiusPointYT.northing;
        ppRadiusAB = mf.yt.ppRadiusYT;
    }
    */
}