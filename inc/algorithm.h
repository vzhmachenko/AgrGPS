#ifndef _Algorithm_H_
#define _Algorithm_H__Periph_H_

void receiveFromDMA(void *param);
int8_t isStringFull(char * str, uint8_t size);
void regToDisplay(uint32_t reg, int8_t strNum);
void tempTask(void *tem);
void tempTask2(void *tem);
void strallcpy(char *to, char *from, int size);
#endif
