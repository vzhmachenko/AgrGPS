#include "main.h"

//		Char buffer variables and pointers
extern char  rxDMAbuf0	[strlen_r];     
extern char  rxDMAbuf1	[strlen_r];
extern char  toBlue		[strlen_t];
char* rxDMA[2] = {	&rxDMAbuf0[0], 
                  	&rxDMAbuf1[0]	};

/*------RTOS variables------*/

QueueHandle_t	  xpQueue;      	///< Указатель на очередь взаимодействия между задачей и прерыванием (dma --> nmea)
QueueHandle_t		lcdQueue;      	///< Указатель на очередь взаимодействия мужду задачами (char --> lcd)
BaseType_t      xStatus;
BaseType_t*     xHigherPriorityTaskWoken;
/*-----------------------------*/

int main(void) {

	/*Hardware initialisation*/
	RCC_Init();
	initAllPeriph();

	xpQueue 	= xQueueCreate(1, sizeof(char*) ); //Создаем очередь
	lcdQueue	= xQueueCreate(2, sizeof(lineParam));      // Максимально хранится 5 структур
	
/* Временные тестовые задачи. */
	xTaskCreate(tempTask,  "temp", 32, NULL, 1, NULL);
	xTaskCreate(tempTask2, "temk", 32, NULL, 1, NULL);

//******************************************************************************//
//******************* Задача получения NMEA сообщений через DMA ****************//
//******************************************************************************//
	xTaskCreate(receiveFromDMA, "NMEAbyDMA",
				300, NULL, 3, NULL);


//******************************************************************************/
//******************* Задача сканирования клавиатуры*********** ****************/
//******************************************************************************/
	xTaskCreate(keyboardScan, "ScanKeyb",
							200, NULL, 1, NULL);


// Запускаем планировщик заданий
	vTaskStartScheduler();
// Бесконечный цикл
	for(;;){
		;
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
		DMA2_Stream7->CR 	&= ~DMA_SxCR_EN;						// DMA -> EN
		DMA2->HIFCR 			|= DMA_HIFCR_CTCIF7;        // Сбросить бит прервания	    
	}
}

/*****************************************************/
/*  
******************************************************/
void
USART6_IRQHandler(){
	char character;
	if(USART6->SR >> 5 & 0x01 != 0){
		character = USART6->DR;
		USART6->DR = character;
	}
}

void initAllPeriph(){
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

}
/* ---------------------------------------------------------- */
