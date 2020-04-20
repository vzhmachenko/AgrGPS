#include "stm_tasks.h"
#include "nmea.h"
#include "ABLine.h"
#include "position.h"


extern	QueueHandle_t		lcdQueue;      	///< Указатель на очередь взаимодействия мужду задачами (char --> lcd)
extern	QueueHandle_t	  xpQueue;      	///< Указатель на очередь взаимодействия между задачей и прерыванием (dma --> nmea)
extern	NMEA						nmea;
extern	ABline         	abline;
extern	Vehicle 				vehicle;
extern	Position 				position;

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
	queue btQueue;									// Создаем очередь сообщений
	btQueue.create = &create;							// Делаем метод-функцию, для ООП
	btQueue.create(&btQueue);							// Инициализируем начальные значения и другие методы-функции

	TaskHandle_t xParseTaskHandle;

	createStartNMEA();
	initVehicle();
	initABl();
	initPosition();


	xTaskCreate(taskParseNMEA, "ParseTask", 300,  		// Создаем задачу обработки NMEA-сообщени.
							&toBlue[0], 3, &xParseTaskHandle);

	uint8_t c = 0;						
	for(;;){
		BaseType_t xStatus1 = xQueueReceive(xpQueue, &receivePointer, 50);		//Receiving the data
		if(xStatus1 == pdPASS){										//Check if data received
			c++;
			GPIOD->ODR ^= 0x8;

			btQueue.push(&btQueue, receivePointer, strlen_r);		//Add data to general queue

			//DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth
			//DMA2_Stream7->CR |= DMA_SxCR_EN;

			//while( findEOS(&btQueue) ){								//If we have a new-line charachter '\n'
				//DMA2_Stream7->NDTR = pop(toBlue, &btQueue);        	//Number of charachters to send by bluetooth 
					//toBlue[0]='\n';		//Or first symbol of toBlue = '$'	//It's fo debugging
				//DMA2_Stream7->CR |= DMA_SxCR_EN;					//Enable transmit by DMA

			while( DMA2_Stream7->NDTR = pop(toBlue, &btQueue)){        	//Number of charachters to send by bluetooth
				DMA2_Stream7->CR |= DMA_SxCR_EN;
				vTaskResume(xParseTaskHandle);					//После запуска просто возобновляем выполнение работы
				GPIOD->ODR ^= 0x800;

				if(abline.flags >> ABLineSet & 0x01){
					//doubleToDisplay(abline.distanceFromCurrentLine, 3);
					addToQueue_doubleToDisplay(lcdQueue, abline.distanceFromCurrentLine, 3);
					addToQueue_doubleToDisplay(lcdQueue, abline.passNumber, 0);
				}
				//LCD_SendCommand(0x01);
			}
		}
	}
}

void 
taskParseNMEA(void *parameter){
  vec3 pivotAxlePos;   //position for AB_Calculations
  while (1) {
    splitString( (char*) parameter);          // Разбиваем сообщение по массивам

    if (strstr( (char*) parameter, "$GPGGA") != NULL) ParseGGA(); 
    if (strstr( (char*) parameter, "$GPVTG") != NULL) ParseVTG();
    if (strstr( (char*) parameter, "$GPRMC") != NULL) ParseRMC();
    if (strstr( (char*) parameter, "$GPGLL") != NULL) ParseGLL();

    // Ошибка в координатах, товыходим
    if( (nmea.flags >> latitudeOk   & 0x01)
    &&  (nmea.flags >> longtitudeOk & 0x01) ) {

      UpdateFixPosition();

//      doubleToDisplay(nmea.latitude,  1);
//      doubleToDisplay(nmea.longitude, 2);
      //print
      addToQueue_doubleToDisplay(lcdQueue, nmea.latitude,  1);
      //print
      addToQueue_doubleToDisplay(lcdQueue, nmea.longitude, 2);

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
		if( ++counter % 4 == 0){
			GPIOB->ODR	&=	~GPIO_PIN_10;
			GPIOE->ODR	|=	GPIO_PIN_14; 

			if( (GPIOE->IDR >> 12) & 0x01){ 	//key=*
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){  //key=0
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){   //key=#
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	  //key=D
				vTaskDelay(300);
        continue;
			}
		}

    	if( counter % 4 == 1){
			GPIOE->ODR	&=	~GPIO_PIN_14;
			GPIOB->ODR	|=	GPIO_PIN_14; 

			if( (GPIOE->IDR >> 12) & 0x01){  //key=1
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=2
				//GPIOD->ODR ^= 0x380;
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=3
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=A
				xTaskCreate(btnAPoint, "APoint", 100, NULL, 3, NULL);
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
				//btnBPoint();
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){    //key=B
				xTaskCreate(btnBPoint, "BPoint", 100, NULL, 3, NULL);
				vTaskDelay(300);
        continue;
			}
		}

		if( counter % 4 == 3){
			GPIOB->ODR	&=	~GPIO_PIN_12;
			GPIOB->ODR	|=	GPIO_PIN_10; 

			if( (GPIOE->IDR >> 12) & 0x01){  //key=7
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 10) & 0x01){   //key=8
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOE->IDR >> 8) & 0x01){    //key=9
				vTaskDelay(300);
        continue;
			}
			if ( (GPIOB->IDR >> 2) & 0x01){	//key=C
				vTaskDelay(300);
        continue;
			}
		}

		vTaskDelay(30);
	}
}

/*
  Отправляем строку на дисплей
*/
void
taskLCD_Send_String(void *prm){
	QueueHandle_t taskQueue = (QueueHandle_t)prm;

	lineParam buff;
	if(xQueueReceive(taskQueue, &buff, 50) != pdPASS){
		vTaskDelete(NULL);
	}

	// Провеляем правильность указания номера строки
	if (buff.lineNumber > 3){
		vTaskDelete(NULL);
	}

	LCD_SendCommandOrData(lineAddr[buff.lineNumber], 0);
	delay_ms(10);//10
	uint8_t i = 0;
	while (buff.string[i] != '\n' 
			&& buff.string[i] != 0 
			&& i < 20)	{		
		LCD_SendCommandOrData(buff.string[i], 1);	
		delay_ms(10);//10
		i++;
	}
	// Добиваем строку пробелами
	while(i < 20) {
		LCD_SendCommandOrData(' ', 1);
		delay_ms(10);//10
		i++;
	}

	vTaskDelete(NULL);
}


/* Наблюдение за очереддю монитора. */
void 
taskLCD_QueueObserver(void *prm){
	uint16_t sizeOfQueue = 0;
	QueueHandle_t taskQueue = (QueueHandle_t)prm;
  for(;;){
		if( (sizeOfQueue = uxQueueMessagesWaiting(taskQueue) ) != 0){
			xTaskCreate(taskLCD_Send_String, "LCD_Task", 64, (void*)taskQueue, 3,  NULL);
		}
		vTaskDelay(300);
  }
}

/* Генерация строк в очередь. */
void  
taskGenStrings(void *prm){
	BaseType_t result;
	lineParam temp;
	uint16_t queueSize = 0x00;

	for(;;){

		strToDisplay(lcdQueue, 0, "Some string 1 line.");
		vTaskDelay(2000);

		strToDisplay(lcdQueue, 1, "Another line 2.");
		vTaskDelay(2000);

		strToDisplay(lcdQueue, 2, "Add one line 3.");
		vTaskDelay(2000);

		strToDisplay(lcdQueue, 3, "4 str.");
		vTaskDelay(1800);

		LCD_SendCommandOrData(0x01, 0);
		vTaskDelay(200);
	}
}

/* ------------------------------------------------- */

/*
 * Дейстивя при нажатии кнопки А
 */
void 
btnAPoint(void *prm){
  // Если линия еще не задавалась, то устанавливаем точку A
  if( !(abline.flags >> ABLineSet & 0x01) )  {

    vec3 fix = position.pivotAxlePos;
    			fix.easting = nmea.fix.easting;
    			fix.northing = nmea.fix.northing;
    			fix.heading = nmea.headingTrue;
    abline.refPoint1.easting  = fix.easting;
    abline.refPoint1.northing = fix.northing;
    abline.abHeading          = fix.heading;

    abline.flags |= 1 << APointSet;    // Выставлям флаг Точки А
    strToDisplay (lcdQueue, 0, "A-point is Set.");
		vTaskDelete(NULL);
  }
  else{
    strToDisplay (lcdQueue, 0, "ABline-Line is set already.");
    strToDisplay (lcdQueue, 1, "Try B to change A-point.");
		vTaskDelete(NULL);
  }
}

/*
 * Дейстивя при нажатии кнопки B
 */
void 
btnBPoint(void* prm){
  //Если не установлена точка А
  if(! (abline.flags >> APointSet & 0x01 )){
    //LCD_Send_String(0, "First set A-Point.");
    strToDisplay (lcdQueue, 0, "First set A-Point.");
		vTaskDelete(NULL);
  //  return;
  }
//<commentingTAG>
/*
  // Если расстояние между точками не достаточно для корректного 
  // определения направления линии
  //x2-x1
  double dx = abline.refABLineP2.easting - abline.refABLineP1.easting;
  //z2-z1
  double dy = abline.refABLineP2.northing - abline.refABLineP1.northing;
  double distInPoint = sqrt( (dx * dx) + (dy * dy));
	if(distInPoint < 0.00001){
    LCD_Send_String(0, "abline-Line error.");
		return;
  }
*/ 
//</commentingTAG>

  // Если линия АБ установлена( установлена точка А и В)
  // то делаем "замещение точек"
  if(abline.flags >> ABLineSet & 0x01){
		abline.refPoint1 = abline.refPoint2;
    lineParam t;
    initLCDstruct (&t, 0, "Changed Points.");
    xQueueSendToBack(lcdQueue, &t, 50);
  }

  abline.refPoint2.easting  = nmea.fix.easting;
  abline.refPoint2.northing = nmea.fix.northing;

/*
	abline.refPoint2.easting = 0.7269;
	abline.refPoint2.northing = 32.4804;
	abline.refPoint1.easting = 0.7269;
	abline.refPoint1.northing = 17.7733;
	*/


  abline.abHeading = atan2(abline.refPoint2.easting  - abline.refPoint1.easting, 
                           abline.refPoint2.northing - abline.refPoint1.northing);
  if (abline.abHeading < 0)
    abline.abHeading += twoPI;

  //sin x cos z for endpoints, opposite for additional lines
  abline.refABLineP1.easting  = abline.refPoint1.easting  - (sin(abline.abHeading) * 1600.0);
  abline.refABLineP1.northing = abline.refPoint1.northing - (cos(abline.abHeading) * 1600.0);

  abline.refABLineP2.easting  = abline.refPoint1.easting  + (sin(abline.abHeading) * 1600.0);
  abline.refABLineP2.northing = abline.refPoint1.northing + (cos(abline.abHeading) * 1600.0);
	

  abline.flags |= 1 << BPointSet;    // Выставлям флаг Точки B
  abline.flags |= 1 << ABLineSet;    // Выставляем флаг линии

  lineParam t;
  initLCDstruct (&t, 0, "B-point is Set.");
  xQueueSendToBack(lcdQueue, &t, 50);
	vTaskDelete(NULL);
}
