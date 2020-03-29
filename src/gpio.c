#include "gpio.h"

/*! Установка таймера 2.*/
void 
timerini2(void){
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 1000-1;	//	= 168000/4
  TIM2->CR1 = TIM_CR1_OPM;
}


/* Задержка, работающая на 2 таймере. */
void 
delay_ms(uint16_t ms){
  TIM2->ARR = ms;		// *= 4
  TIM2->CNT = 0;
  TIM2->CR1 = TIM_CR1_CEN;

  while((TIM2->SR & TIM_SR_UIF)==0) {;}

  TIM2->SR &= ~TIM_SR_UIF;
}


/*! Устанавливаем значение пина. */
void 
GPIO_WritePin(GPIO_TypeDef  *GPIOx, 
              uint16_t      GPIO_Pin, 
              FlagStatus    PinState) {
  if(PinState != RESET)  
    GPIOx->BSRRL = GPIO_Pin;
  else  
    GPIOx->BSRRH = GPIO_Pin;
}


/*! Запись данных на ножки дисплея. */
void 
LCD_Set_Data(uint8_t data){
  GPIOE->BSRRL =  (0x0000 | data);      ///Set Bits
  GPIOE->BSRRH = ~(0xFFFF & data);      /// ReSet Bits
}


/* Отправляем команду на дисплей. */
void 
LCD_SendCommandOrData(uint8_t data, uint8_t command){
  //if data     command == 1
  //if command  command == 0
  RS(command);
  LCD_Set_Data(data);
  EN(1);
  delay_ms(1);//10
  EN(0);
}


/*! Процедура инициализации дисплея. */
void 
LCD_ini(void){
	RW(0);
  uint8_t commands[6] = {0x38, 0x38, 0x0F, 0x01, 0x06, 0x02};
  for(uint8_t i = 0; i < 6; ++i){
    LCD_SendCommandOrData(commands[i], 0);
    delay_ms(50);
  }
}


/* Отправляем строку на дисплей. */
void
LCD_Send_String(uint8_t String_Num, char* str){
  // Провеляем правильность указания номера строки.
  if (String_Num > 3)
    return;

  // Посылаем адрес строки, в которую будем писать.
  LCD_SendCommandOrData(lineAddr[String_Num], 0);
	delay_ms(10);//10

	uint8_t i = 0;
	while (str[i] != '\n' 
      && str[i] != 0 
      && i < 20)	{		
		LCD_SendCommandOrData(str[i], 1);	
		delay_ms(10);
		i++;
	}

  // Добиваем строку пробелами
  while(i < 20) {
    LCD_SendCommandOrData(' ', 1);
    delay_ms(10);
    i++;
  }
}

/*
 * Вывод на дисплей значение регистра (32bit-value)
*/
void 
regToDisplay(uint32_t reg, int8_t strNum){
	char bufer[9];
	bufer[8] = 0;
	for(uint8_t i = 0; i < 8; i++){
		uint32_t temp = reg >> (4*(7-i)) & 0xF;
		bufer[i] = temp < 10 
             ? '0' + (int)temp 
             : 'A' + (int)temp - 10;
	}
	LCD_Send_String(strNum, bufer);
}

/*
 * Вывод на дисплей числа с запятой
*/
void 
doubleToDisplay(double num, int8_t strNum){
	char lengthToLine[9] = {0};
	itoa( (int)num, lengthToLine, 10); 	//При необходимости умножить для повышения точности
	uint8_t strL = strlen(lengthToLine);
	lengthToLine[strL] = '.';
	int lBytes = (num - (int) num) * 100000;
	itoa(lBytes, lengthToLine + strL + 1, 10); 	//При необходимости умножить для повышения точности

	LCD_Send_String(strNum, lengthToLine);
}


void initLCDstruct(lineParam* line, uint8_t string_num, char *str){
  line->lineNumber = string_num;
  memset(line->string, 0, 30);
  strncpy(line->string, str, 29);
}
