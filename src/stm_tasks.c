#include "stm_tasks.h"
#include "nmea.h"
#include "ABLine.h"
#include "position.h"


extern 	QueueHandle_t		lcdQueue;      	///< Указатель на очередь взаимодействия мужду задачами (char --> lcd)
extern	QueueHandle_t	  xpQueue;      	///< Указатель на очередь взаимодействия между задачей и прерыванием (dma --> nmea)
extern 	NMEA 						nmea;
extern	ABline         	abline;
extern 	Vehicle 				vehicle;
extern 	Position 				position;

extern char  toBlue		[strlen_t];
char *receivePointer;


/*! Задача миганием светлодиодом. */
void 
tempTask(void *tem){
	while(1){
		GPIOD->ODR ^= 0x10;
		vTaskDelay(40);
	}
}

/*! Задача миганием светлодиодом. */
void 
tempTask2(void *tem){
	while(1){
		GPIOD->ODR ^= 0x1000;
		vTaskDelay(400);
	}
}

/* Задача чтения NMEA-сообщений по DMA */
void 
receiveFromDMA(void *param){
	static queue btQueue;									// Создаем очередь сообщений
	TaskHandle_t xParseTaskHandle;
	TaskHandle_t lcdTaskHandler;
	TaskHandle_t lcdTaskObserver;

	btQueue.create = &create;							// Делаем метод-функцию, для ООП
	btQueue.create(&btQueue);							// Инициализируем начальные значения и другие методы-функции

	createStartNMEA();
	initVehicle();
	initABl();
	initPosition();


	xTaskCreate(taskParseNMEA, "ParseTask", 200,  		// Создаем задачу обработки NMEA-сообщени.
							&toBlue[0], 3, &xParseTaskHandle);
							
	//void* arr[2] = {(void*)(&lcdQueue), (void*)(&lcdTaskHandler)};
	//xTaskCreate( taskLCD_QueueObserver, "lcdObserv", 40,
	//						arr, 2, &lcdTaskObserver);


	for(;;){
		BaseType_t xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);		//Receiving the data
		if(xStatus1 == pdPASS){										//Check if data received
			btQueue.push(&btQueue, receivePointer, strlen_r);		//Add data to general queue

			DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth
			DMA2_Stream7->CR |= DMA_SxCR_EN;

			while( findEOS(&btQueue) ){								//If we have a new-line charachter '\n'
				DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth 
				//toBlue[0]='\n';		//Or first symbol of toBlue = '$'	//It's fo debugging
				DMA2_Stream7->CR |= DMA_SxCR_EN;					//Enable transmit by DMA

				vTaskResume(xParseTaskHandle);					//После запуска просто возобновляем выполнение работы

				if(abline.flags >> ABLineSet & 0x01)
					doubleToDisplay(abline.distanceFromCurrentLine, 3);


				//LCD_SendCommand(0x01);
			}
		}
	}
}

void 
taskParseNMEA(void *parameter){
  vec3 pivotAxlePos;   //position for AB_Calculations
  while (1) {
    doubleToDisplay(21.143,  3);
    splitString( (char*) parameter);          // Разбиваем сообщение по массивам

    if (strstr( (char*) parameter, "$GPGGA") != NULL) ParseGGA(); 
    if (strstr( (char*) parameter, "$GPVTG") != NULL) ParseVTG();
    if (strstr( (char*) parameter, "$GPRMC") != NULL) ParseRMC();
    if (strstr( (char*) parameter, "$GPGLL") != NULL) ParseGLL();

    // Ошибка в координатах, товыходим
    if( (nmea.flags >> latitudeOk   & 0x01)
    &&  (nmea.flags >> longtitudeOk & 0x01) ) {

      UpdateFixPosition();

      doubleToDisplay(nmea.latitude,  1);
      doubleToDisplay(nmea.longitude, 2);

      if (abline.flags >> ABLineSet & 0x01) {
        pivotAxlePos.easting  = nmea.fix.easting - (sin(position.pivotAxlePos.heading) 
                              * vehicle.antennaPivot);
        pivotAxlePos.northing = nmea.fix.easting - (cos(position.pivotAxlePos.heading) 
                              * vehicle.antennaPivot);
        pivotAxlePos.heading  = position.fixHeading;

        GetCurrentABLine(pivotAxlePos);
      }
    }

    // Очищаем флаги для предотвращения обработки повторных данных
    nmea.flags &= ~(0x01 << latitudeOk);
    nmea.flags &= ~(0x01 << longtitudeOk);
    vTaskSuspend(NULL);         // При завершении обработки сообщения приостанавливаем задачу
                                // Для ожидания нового сообщения для обработки
  }
}

/* Сканирование клавиатуры */
void 
keyboardScan(void *param){
	static uint8_t counter = 0;
	while(1) {
		if( counter % 4 == 0){
			GPIOB->ODR	&=	~GPIO_PIN_10;
			GPIOE->ODR	|=	GPIO_PIN_14; 

			if( (GPIOE->IDR >> 12) & 0x01){ 	//key=*
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){  //key=0
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){   //key=#
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	  //key=D
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
		}

    	if( counter % 4 == 1){
			GPIOE->ODR	&=	~GPIO_PIN_14;
			GPIOB->ODR	|=	GPIO_PIN_14; 

			if( (GPIOE->IDR >> 12) & 0x01){  //key=1
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=2
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=3
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=A
				btnAPoint();
				vTaskDelay(300);
        continue;
			}
	 	}

		if( counter % 4 == 2){
			GPIOB->ODR	&=	~GPIO_PIN_14;
			GPIOB->ODR	|=	GPIO_PIN_12;

			if( (GPIOE->IDR >> 12) & 0x01){  //key=4
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=5
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=6
				GPIOD->ODR ^= 0x380;
				btnBPoint();
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=B
				btnBPoint();
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
		}

		if( counter % 4 == 3){
			GPIOB->ODR	&=	~GPIO_PIN_12;
			GPIOB->ODR	|=	GPIO_PIN_10; 

			if( (GPIOE->IDR >> 12) & 0x01){  //key=7
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=8
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=9
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	//key=C
				GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
		}

		if(++counter == 4)
			counter = 0;    

		vTaskDelay(30);
	}
}

/*
  Отправляем строку на дисплей
*/
void
taskLCD_Send_String(void *prm){
  lineParam buff;
  for(;;){
    if(xQueueReceive(lcdQueue, &buff, 50) != pdPASS){
      vTaskDelete(NULL);
    }

    // Провеляем правильность указания номера строки
    if (buff.lineNumber > 3){
      vTaskDelete(NULL);
    }
    LCD_SendCommandOrData(lineAddr[buff.lineNumber], 0);
    vTaskDelay(100);
    uint8_t i = 0;
    while (buff.string[i] != '\n' 
        && buff.string[i] != 0 
        && i < 20)	{		
      LCD_SendCommandOrData(buff.string[i], 1);	
      vTaskDelay(100);
      i++;
    }
    // Добиваем строку пробелами
    while(i < 20) {
      LCD_SendCommandOrData(' ', 1);
      vTaskDelay(100);
      i++;
    }
      vTaskDelete(NULL);
  }
}

/* Наблюдение за очереддю монитора. */
void 
taskLCD_QueueObserver(void *prm){
  for(;;){
		if(uxQueueMessagesWaiting(lcdQueue) != 0){
			xTaskCreate(taskLCD_Send_String, "LCD_Task", 40,
									NULL, 3,  NULL);
		}
		vTaskDelay(100);
  }
}
/* ------------------------------------------------- */