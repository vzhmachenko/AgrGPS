#include "stm32f4xx.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "algorithm.h"
#include "gpio.h"
#include "periph.h"
#include <charQueue.h>

//#include "vehicle.h"
#include "nmea.h"

char recString0[strlen_r];     //this will hold the recieved stringw
char recString1[strlen_r];
char toBlue[strlen_t];
char *arStr[2] = {&recString0[0], &recString1[0]};
/*------RTOS variables------*/
QueueHandle_t	xpQueue;
BaseType_t xStatus;
BaseType_t xHigherPriorityTaskWoken;
/*-----------------------------*/
int main(void)
{
/*Hardware initialisation*/
	RCC_Init();
	gpio_ini();
	//GPIOD->ODR= 0xF;
	timerini2();
	LCD_ini();
	delay_ms(20);
	LCD_ini();
	USART2_init();
	USART6_init();
	dma1ini();
	dma2ini();
	__enable_irq();

/*----------------------*/

/*----------------------*/
	xpQueue = xQueueCreate(1, sizeof(char *));

/*----------------------*/
	

	xTaskCreate( receiveFromDMA, "Receive ", 800, NULL, 3, NULL);
	xTaskCreate(tempTask, "temp", 30, NULL, 1, NULL);
	xTaskCreate(tempTask2, "temdp", 30, NULL, 1, NULL);
	xTaskCreate(keyboardScan, "Keybr", 30, NULL, 1, NULL);
	vTaskStartScheduler();

	for(;;){

	}
}

void DMA1_Stream5_IRQHandler(){
	xHigherPriorityTaskWoken = pdFALSE;
	if( (DMA1->HISR & DMA_HISR_TCIF5 ) == DMA_HISR_TCIF5){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;        //Сбросить бит прервания
		xStatus = 0;
		if(	(DMA1_Stream5->CR & DMA_SxCR_CT) == DMA_SxCR_CT){
			//if CT bit == 1 -> Memory 1 write mode
			//have to send Memory 0
			xStatus = xQueueSendToBackFromISR(xpQueue, &arStr[0], xHigherPriorityTaskWoken);
		}
		else {
			xStatus = xQueueSendToBackFromISR(xpQueue, &arStr[1], xHigherPriorityTaskWoken);
		}				
		if(xStatus != pdPASS) 
			;
			//LCD_Send_String(3, "DMA_Error!");
		else
			;
			//LCD_Send_String(3, "DMAsend OK");
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}
}

void DMA2_Stream7_IRQHandler(){
	if( (DMA2->HISR & DMA_HISR_TCIF7 ) == DMA_HISR_TCIF7){
		DMA2_Stream7->CR 	&= ~DMA_SxCR_EN;		//DMA -> EN
		DMA2->HIFCR 		|= DMA_HIFCR_CTCIF7;        //Сбросить бит прервания	    
	}
}
void USART6_IRQHandler(){
	char t;
	if(USART6->SR >> 5 &0x01 != 0){
		t = USART6->DR;
		USART6->DR = t;
	}
}
