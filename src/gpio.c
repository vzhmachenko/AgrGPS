#include "gpio.h"
// Адреса начальных символов строк
static uint8_t rowAdr[4] = {0x80, 0xc0, 0x94, 0xd4};

/*!
  Установка таймера 2
*/
void 
timerini2(void){
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 1000-1;	//	= 168000/4
  TIM2->CR1 = TIM_CR1_OPM;
}

/*
  Задержка, работающая на 2 таймере
*/
void 
delay_ms(uint16_t ms){
  TIM2->ARR = ms;		// *= 4
  TIM2->CNT = 0;
  TIM2->CR1 = TIM_CR1_CEN;

  while((TIM2->SR & TIM_SR_UIF)==0) {;}

  TIM2->SR &= ~TIM_SR_UIF;
}

/*!
  Устанавливаем значение пина
*/
void 
GPIO_WritePin(GPIO_TypeDef  *GPIOx, 
              uint16_t      GPIO_Pin, 
              FlagStatus    PinState) {
  if(PinState != RESET)  
    GPIOx->BSRRL = GPIO_Pin;
  else  
    GPIOx->BSRRH = GPIO_Pin;
}

/*!
  Запись данных на ножки дисплея
*/
void 
LCD_Set_Data(uint8_t data){
  GPIOE->BSRRL =  (0x0000 | data);      ///Set Bits
  GPIOE->BSRRH = ~(0xFFFF & data);      /// ReSet Bits
}

/*
  Отправляем команду на дисплей
*/
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


/*!
  Процедура инициализации дисплея
*/
void 
LCD_ini(void){
	RW(0);
  uint8_t commands[6] = {0x38, 0x38, 0x0F, 0x01, 0x06, 0x02};
  for(uint8_t i = 0; i < 6; ++i){
    LCD_SendCommandOrData(commands[i], 0);
    delay_ms(50);
  }
}

/*
  Отправляем строку на дисплей
*/
uint8_t 
LCD_Send_String(uint8_t String_Num, char* str){
//  static uint8_t counter = 0;
  //static uint8_t busy = 0;
//  if(busy)
//    return 0 ;
//
//  busy = 1;

  // Провеляем правильность указания номера строки
  if (String_Num > 3)
    return 0;

  LCD_SendCommandOrData(rowAdr[String_Num], 0);
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
 // busy = 0;
	return i;
}
