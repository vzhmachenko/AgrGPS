#ifndef _POSITION_H_
#define _POSITION_H_

#include "nmea.h"
#include <math.h>
typedef struct {
    
        int isGPSPositionInitialized;   //bool
        int isFirstFixPositionSet;      //bool
        int startCounter;
        int totalFixSteps;
        int currentStepFixcurre

} position;

void setInitPostitionParameters(void);
void UpdateFixPosition(void);
 #endif
