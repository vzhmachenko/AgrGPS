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
extern char toBlue[200];

BaseType_t 		xStatus1;
void receiveFromDMA(void *param){
	toBlue[0] = '%';
	static uint8_t newLinePos = 0;
	static uint8_t sizeOfStr = 0;
	for(;;){		
		xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);
		if(xStatus1 == pdPASS){
			sizeOfStr += strlen;
			strallcpy(toBlue +1+ newLinePos, receivePointer, strlen);
			while ( (newLinePos = isStringFull(toBlue+1, sizeOfStr)) != 0 ){

				DMA2_Stream7->NDTR = newLinePos-1;        // Количество даних для передачи 
				DMA2_Stream7->CR |= DMA_SxCR_EN;		//DMA -> EN

				strallcpy(toBlue+1, toBlue+newLinePos+1, 
						sizeOfStr - newLinePos);
				sizeOfStr -= newLinePos-1;
			}
			newLinePos = sizeOfStr+1;



//			LCD_Send_String(0, receivePointer);
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

int8_t isStringFull(char *str, uint8_t size){
	for(uint8_t i = 0; i < size; i++)
		if(*(str+i) == 0)
			return i+1;

	return 0;
}
void strallcpy(char *to, char *from, int size){
	for(int i = 0; i<size; i++)
		*(to+i) = *(from+i);
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
