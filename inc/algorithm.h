#ifndef _Algorithm_H_
#define _Algorithm_H_

void receiveFromDMA(void *param);
void regToDisplay(uint32_t reg, 
                  int8_t strNum);
void doubleToDisplay( double num,  
                      int8_t strNum);
void tempTask(void *tem);
void tempTask2(void *tem);
void keyboardScan(void *param);
#endif
