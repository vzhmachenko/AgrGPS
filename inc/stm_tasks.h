#ifndef _STM_TASKS_
#define _STM_TASKS_

#include "gpio.h"
#include "vehicle.h"
#include "charQueue.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"



void  tempTask(void *tem);

void  tempTask2(void *tem);

void  receiveFromDMA(void *param);

void  taskParseNMEA(void *parameter);

void  keyboardScan(void *param);

void  taskLCD_Send_String(void *prm);

void  taskLCD_QueueObserver(void *prm);

#endif
