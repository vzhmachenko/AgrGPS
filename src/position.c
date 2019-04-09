#include "position.h"

position pos;
extern pn;


void setInitPostitionParameters(){
    pos.isFirstFixPositionSet = 0;
    pos.isGPSPositionInitialized = 0;
    
    pos.startCounter = 0;
    pos.totalFixSteps = 5 * 6;
}
void UpdateFixPosition(void){
    if(pos.isGPSPositionInitialized == 0)
        InitializeFirstFewGPSPositions();
    //region Antenne Offset
}