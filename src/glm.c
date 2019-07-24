#include "glm.h"
#include "vehicle.h"
#include <math.h>

double 
toRadians(double degrees){
    return degrees * 0.01745329251994329576923690766743;
}

double 
toDegrees (double radians){
    return radians * 57.295779513082325225835265587528;
}

double 
DistanceVec2Vec2(vec2 first, vec2 second){
    return sqrt(pow(first.easting - second.easting, 2) + 
        pow(first.northing - second.northing, 2)); 
}

double 
DistanceVec2Vec3(vec2 first, vec3 second){
    return sqrt(pow(first.easting - second.easting, 2) + 
        pow(first.northing - second.northing, 2)); 
}

double 
DistanceVec3Vec3(vec3 first, vec3 second){
    return sqrt(pow(first.easting - second.easting, 2) + 
        pow(first.northing - second.northing, 2)); 
}

double 
DistanceSquared(double northing1, double easting1, double northing2, double easting2) {
    return pow(easting1 - easting2, 2) + pow(northing1 - northing2, 2);
}