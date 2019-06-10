#include "stm32f4xx.h"
#include "algorithm.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

#include "queue.h"
#include "charQueue.h"

#include "nmea.h"
#include "position.h"
#include "ABLine.h"
extern 	QueueHandle_t	xpQueue;
extern 	char 			toBlue[strlen_t];
		char 		   *receivePointer;
		BaseType_t 		xStatus1;
extern ABline AB;
extern NMEA pn;

void receiveFromDMA(void *param){
	static queue btQueue;		//Создаем очередь сообщений
	TaskHandle_t xParseTaskHandle = NULL;
	btQueue.create = &create;	//Делаем метод-функцию, для ООП
	btQueue.create(&btQueue);	//Инициализируем начальные значения и другие методы-функции
	createStartNMEA();
	initVehicle();
	initABl();
	initPosition();
	for(;;){
		xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);		//Receiving the data
		if(xStatus1 == pdPASS){										//Check if data received
			btQueue.push(&btQueue, receivePointer, strlen_r);		//Add data to general queue

			DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth
			DMA2_Stream7->CR |= DMA_SxCR_EN;
			xTaskCreate(ParseNMEA, "ParseTask", 200,  		//
					&toBlue[0], 3, &xParseTaskHandle);

			while( findEOS(&btQueue) ){								//If we have a new-line charachter '\n'
				DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth 
				//toBlue[0]='\n';		//Or first symbol of toBlue = '$'	//It's fo debugging
				DMA2_Stream7->CR |= DMA_SxCR_EN;					//Enable transmit by DMA

				vTaskResume(xParseTaskHandle);					//После запуска просто возобновляем выполнение работы
				if(AB.isABLineSet != 0)
					doubleToDisplay(AB.distanceFromCurrentLine, 3);

				doubleToDisplay(pn.zone, 1);
				doubleToDisplay(pn.fix.easting, 2);
				doubleToDisplay(pn.fix.northing, 3);

				//LCD_SendCommand(0x01);
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
		GPIOD->ODR ^= 0x10;
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
void doubleToDisplay(double num, int8_t strNum){
	char lengthToLine[9];
	itoa( (int)num, lengthToLine, 10); 	//При необходимости умножить для повышения точности
	LCD_Send_String(strNum, lengthToLine);
}

void keyboardScan(void *param){
	static uint8_t counter = 0;
	while(1){

		if(counter%4 == 0){
			GPIOB->ODR	&=	~GPIO_PIN_10;
			GPIOE->ODR	|=	GPIO_PIN_14; 
			if( (GPIOE->IDR >> 12) & 0x01){ 	//key=*
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=0
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=#
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	//key=D
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
        }

    	if(counter%4 == 1){
			GPIOE->ODR	&=	~GPIO_PIN_14;
			GPIOB->ODR	|=	GPIO_PIN_14; 
			if( (GPIOE->IDR >> 12) & 0x01){  //key=1
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=2
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=3
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=A
				btnAPoint();
				vTaskDelay(300);
			}
	 	}

   		if(counter%4 == 2){
			GPIOB->ODR	&=	~GPIO_PIN_14;
			GPIOB->ODR	|=	GPIO_PIN_12;
			if( (GPIOE->IDR >> 12) & 0x01){  //key=4
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=5
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=6
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=B
				btnBPoint();
				vTaskDelay(300);
			}
		}

		if(counter%4 == 3){
			GPIOB->ODR	&=	~GPIO_PIN_12;
			GPIOB->ODR	|=	GPIO_PIN_10; 
			if( (GPIOE->IDR >> 12) & 0x01){  //key=7
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=8
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=9
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	//key=C
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
			}
		}

		if(++counter == 4)
			counter = 0;    
		vTaskDelay(30);
	}
}
