#include "gpio.h"

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
  (((data >> 7 ) &0x01) == 1) ? DB7(1) : DB7(0);
  (((data >> 6 ) &0x01) == 1) ? DB6(1) : DB6(0);
  (((data >> 5 ) &0x01) == 1) ? DB5(1) : DB5(0);
  (((data >> 4 ) &0x01) == 1) ? DB4(1) : DB4(0);
  (((data >> 3 ) &0x01) == 1) ? DB3(1) : DB3(0);
  (((data >> 2 ) &0x01) == 1) ? DB2(1) : DB2(0);
  (((data >> 1 ) &0x01) == 1) ? DB1(1) : DB1(0);
  (((data >> 0 ) &0x01) == 1) ? DB0(1) : DB0(0);
}

/*
  Отправляем команду на дисплей
*/
void 
LCD_SendCommand(uint8_t data){
  RS(0);
  LCD_Set_Data(data);
  EN(1);
  delay_ms(1);//10
  EN(0);
}

/*!
  Отправляем данные на дисплей
*/
void 
LCD_SendData(uint8_t data){
  RS(1);
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
    LCD_SendCommand(commands[i]);
    delay_ms(50);
  }
}

/*
  Отправляем строку на дисплей
*/
uint8_t 
LCD_Send_String(uint8_t String_Num, char* str){
  // Провеляем правильность указания номера строки
  if (String_Num > 3)
    return 0;

  // Адреса начальных символов строк
	uint8_t rowAdr[4] = {0x80, 0xc0, 0x94, 0xd4};

  LCD_SendCommand(rowAdr[String_Num]);
	delay_ms(10);//10

	uint8_t i = 0;
	while (str[i] != '\n' && str[i] != 0 && i < 20)	{		
		LCD_SendData(str[i]);	
		delay_ms(10);//10
		i++;
	}

	return i;
}
