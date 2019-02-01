#include "stm32f4xx.h"
#include "algorithm.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>

#define strlen 80
extern char 			bufferedData[5*strlen];
extern QueueHandle_t	xpQueue;
char 					*receivePointer;

BaseType_t 		xStatus1;
void receiveFromDMA(void *param){
	char recBuf[strlen];
	static uint8_t bufferedPosition = 0;
	static uint8_t bufferedSize = 0;
	char buf[5]; static int i = 0;
	for(;;){
		
		xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);
		if(xStatus1 == pdPASS){
			LCD_Send_String(0, receivePointer);
			/*bufferedSize += strlen;
			strncpy(bufferedData + bufferedPosition, receivePointer, strlen);
			while ( isStringFull(bufferedData, bufferedSize) ){
				bufferedPosition = LCD_Send_String(0, bufferedData);
				strncpy(bufferedData, bufferedData+bufferedPosition+1, 
						bufferedSize - bufferedPosition);
				bufferedSize -= bufferedPosition;
				bufferedPosition = bufferedSize + 1; 
			}*/
		}
		else{
			LCD_Send_String(3, "Not passed.");

		}
	}
}

void tempTask2(void *tem){
	while(1){
		GPIOD->ODR ^= 0xF000;
		vTaskDelay(400);
	}
}
void tempTask(void *tem){
	while(1){
		GPIOD->ODR ^= 0xF;
		vTaskDelay(40);
	}
}

int8_t isStringFull(char * str, uint8_t size){
	for(int i = 0; i < size; i++)
		if(*(str+i) == '\0')
			return 1;
	return 0;
}
void regToDisplay(uint32_t reg, int8_t strNum){
	char bufer[9];
	uint8_t k;
	bufer[8] = 0;
	for(uint8_t i = 0; i<8; i++){
		uint32_t temp = reg >> (4*(7-i)) & 0xF;
		if( temp < 10)
			bufer[i] = '0' + (int)temp;
		else
			bufer[i] = 'A' + (int)temp-10;
		}
	LCD_Send_String(strNum, bufer);
}
