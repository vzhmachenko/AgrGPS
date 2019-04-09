#ifndef _GLM_H_
#define _GLM_H_
#define twoPI 6.28318530717958647692
#define  in2m 0.0254;
#define  m2in 39.3701;
#define  m2ft 3.28084;
#define  ha2ac 2.47105;
#define  ac2ha 0.404686;
#define  m2ac 0.000247105;
#define  m2ha 0.0001;
#define  galAc2Lha 9.35396;
#define  LHa2galAc 0.106907;
#define  L2Gal 0.264172;
#define  Gal2L 3.785412534258;
#define  twoPI 6.28318530717958647692;
#define  PIBy2 1.57079632679489661923;

double toRadians(double degrees);
double toDegrees (double radians);
double DistanceVec2Vec2(vec2 first, vec2 second);
double DistanceVec2Vec3(vec2 first, vec3 second);
double DistanceVec3Vec3(vec3 first, vec3 second);
void InitializeFirstFewGPSPositions(void);




 #endif
