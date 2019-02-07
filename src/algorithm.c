#include "stm32f4xx.h"
#include "algorithm.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "charQueue.h"

extern 	QueueHandle_t	xpQueue;
extern 	char 			toBlue[strlen_t];
		char 		   *receivePointer;
		BaseType_t 		xStatus1;

void receiveFromDMA(void *param){
	static queue btQueue;
	btQueue.create = &create;
	btQueue.create(&btQueue);
	for(;;){
		xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);		//Receiving the data
		if(xStatus1 == pdPASS){										//Check if data received
			btQueue.push(&btQueue, receivePointer, strlen_r);		//Add data to general queue
			while( findEOS(&btQueue) ){								//If we have a new-line charachter '\n'
				DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth 
				toBlue[0]='\n';		//Or first symbol of toBlue = '$'	//It's fo debugging
				DMA2_Stream7->CR |= DMA_SxCR_EN;					//Enable transmit by DMA
			}
		}
	}

}

void tempTask2(void *tem){
	while(1){
		GPIOD->ODR ^= 0x1000;
		vTaskDelay(400);
	}
}
void tempTask(void *tem){
	while(1){
		GPIOD->ODR ^= 0x2;
		vTaskDelay(40);
	}
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
