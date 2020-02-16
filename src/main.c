#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "charQueue.h"
#include "algorithm.h"
#include "gpio.h"
#include "periph.h"
#include "nmea.h"

//		Char buffer variables and pointers
char  rxDMAbuf0	[strlen_r];     
char  rxDMAbuf1	[strlen_r];
char  toBlue		[strlen_t];
char* rxDMA[2] = {	&rxDMAbuf0[0], 
                  	&rxDMAbuf1[0]	};

/*------RTOS variables------*/
QueueHandle_t	  xpQueue;      //Указатель на очередь взаимодействия между задачей и прерыванием
BaseType_t      xStatus;
BaseType_t*     xHigherPriorityTaskWoken;
/*-----------------------------*/

int main(void) {
/*Hardware initialisation*/
	RCC_Init();

	gpio_ini();
	timerini2();
	LCD_ini();
	delay_ms(20);
	LCD_ini();
	LCD_Send_String(0, "LCD initization...");

	USART2_init();
	USART6_init();

	dma1ini();
	dma2ini();

	__enable_irq();


	xpQueue = xQueueCreate(1, sizeof(char*) ); //Создаем очередь


//******************************************************************************//
//******************* Задача получения NMEA сообщений через DMA ****************//
//******************************************************************************//
	if( xTaskCreate( receiveFromDMA, "NMEAbyDMA", 
				900, NULL, 3, NULL) != pdPASS)
      GPIOD->ODR |= 0xFFFFFFFF;

//******************************************************************************//
//******************* Задача сканирования клавиатуры*********** ****************//
//******************************************************************************//
	if(xTaskCreate(keyboardScan, "ScanKeyb", 
				600, NULL, 1, NULL) != pdPASS)
      GPIOD->ODR |= 0xFFFFFFFF;


// Временные тестовые задачи
	xTaskCreate(tempTask, 	"temp", 30, NULL, 1, NULL);
	xTaskCreate(tempTask2, 	"temk", 30, NULL, 1, NULL);


// Запускаем планировщик заданий
	vTaskStartScheduler();
// Бесконечный цикл
	for(;;){

	}
}

/*****************************************************/
/*  Обработка прерывания DMA
/*  Получение NMEA-сообщений от GPS-приемника
******************************************************/
void 
DMA1_Stream5_IRQHandler(){
	*xHigherPriorityTaskWoken = pdFALSE;
	if( (DMA1->HISR & DMA_HISR_TCIF5 ) == DMA_HISR_TCIF5){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;        //Сбросить бит прервания
		xStatus = 0;
		if(	(DMA1_Stream5->CR & DMA_SxCR_CT) == DMA_SxCR_CT){
			//if CT bit == 1 -> Memory 1 write mode
			//have to send Memory 0
			xStatus = xQueueSendToBackFromISR(xpQueue, &rxDMA[0], 
                                  xHigherPriorityTaskWoken);
		}
		else {
			xStatus = xQueueSendToBackFromISR(xpQueue, &rxDMA[1], 
                                  xHigherPriorityTaskWoken);
		}				
		if(xStatus != pdPASS) //Произошла ошибка при отправке данных
			;
			//LCD_Send_String(3, "DMA_Error!");
		else
			;
			//LCD_Send_String(3, "DMAsend OK");

    // Если при отправке данных, появилась более приоритетная
    // задача, то принудудительно и недмедленно переключаемся
    // на нее при выходе из прервывания.
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/*****************************************************/
/*  
******************************************************/
void 
DMA2_Stream7_IRQHandler(){
	if( (DMA2->HISR & DMA_HISR_TCIF7 ) == DMA_HISR_TCIF7){
		DMA2_Stream7->CR 	&= ~DMA_SxCR_EN;		//DMA -> EN
		DMA2->HIFCR 		|= DMA_HIFCR_CTCIF7;        //Сбросить бит прервания	    
	}
}

/*****************************************************/
/*  
******************************************************/
void
USART6_IRQHandler(){
	char t;
	if(USART6->SR >> 5 &0x01 != 0){
		t = USART6->DR;
		USART6->DR = t;
	}
}
