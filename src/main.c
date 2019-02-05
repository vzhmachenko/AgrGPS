#include "stm32f4xx.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "algorithm.h"
#include "gpio.h"
#include "periph.h"
#include <stdlib.h>

#define strlen 80            //Максимальная длина строки
char recString0[strlen];     //this will hold the recieved stringw
char recString1[strlen];
char toBlue[200];
char *arStr[2] = {&recString0[0], &recString1[0]};
char bufferedData[2*strlen];
char recReg[9]="12345678";
QueueHandle_t	xpQueue;
BaseType_t xStatus;
BaseType_t xHigherPriorityTaskWoken;

int main(void)
{
/*Hardware initialisation*/
	RCC_Init();
	gpio_ini();
	GPIOD->ODR= 0xF;
	timerini2();
	LCD_ini();
	delay_ms(20);
	LCD_ini();
	xpQueue = xQueueCreate(1, sizeof(char *));

	__enable_irq();
	USART2_init();
	USART6_init();
	dma1ini();
	dma2ini();
/*----------------------*/
	

	xTaskCreate( receiveFromDMA, "Receive ", 500, NULL, 2, NULL);
	xTaskCreate(tempTask, "temp", 30, NULL, 1, NULL);
	xTaskCreate(tempTask2, "temdp", 30, NULL, 1, NULL);
	
	vTaskStartScheduler();

	for(;;){

	}
}

void DMA1_Stream5_IRQHandler(){
	xHigherPriorityTaskWoken = pdFALSE;
	if( (DMA1->HISR & DMA_HISR_TCIF5 ) == DMA_HISR_TCIF5){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;        //Сбросить бит прервания
		xStatus = 0;
		//regToDisplay(arStr[0],0);
		if(	(DMA1_Stream5->CR & DMA_SxCR_CT) == DMA_SxCR_CT){
			//if CT bit == 1 -> Memory 1 write mode
			//have to send Memory 0
			xStatus = xQueueSendToBackFromISR(xpQueue, &arStr[0], xHigherPriorityTaskWoken);
			//DMA2_Stream7->CR 	|= DMA_SxCR_EN;		//DMA -> EN  
		}
		else {
			xStatus = xQueueSendToBackFromISR(xpQueue, &arStr[1], xHigherPriorityTaskWoken);
			//DMA2_Stream7->CR 	|= DMA_SxCR_EN;		//DMA -> EN  
		}				
		//DMA1->HIFCR |= DMA_HIFCR_CTCIF5;        //Сбросить бит прервания
		if(xStatus != pdPASS) ;
			//LCD_Send_String(3, "DMA_Error!");
		else
		{
			//LCD_Send_String(3, "DMAsend OK");
		}
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}
}

void DMA2_Stream7_IRQHandler(){
	if( (DMA2->HISR & DMA_HISR_TCIF7 ) == DMA_HISR_TCIF7){
		DMA2_Stream7->CR 	&= ~DMA_SxCR_EN;		//DMA -> EN
		/*if(	(DMA1_Stream5->CR & DMA_SxCR_CT) == DMA_SxCR_CT){
			DMA2_Stream7->M0AR = (uint32_t)&recString0; 	
		}
		else{
			DMA2_Stream7->M0AR = (uint32_t)&recString1; 	
		}*/
//		DMA2_Stream7->NDTR   = 80;
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
