#include "gpio.h"

void timerini2(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 1000-1;	//	= 168000/4
    TIM2->CR1 = TIM_CR1_OPM;
}
void delay_ms(uint16_t ms){
    TIM2->ARR = ms;		// *= 4
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;

    while((TIM2->SR & TIM_SR_UIF)==0) {;}
    TIM2->SR &= ~TIM_SR_UIF;
}

void GPIO_WritePin(GPIO_TypeDef* GPIOx, 
		uint16_t GPIO_Pin, FlagStatus PinState) {
  if(PinState != RESET)  {
    GPIOx->BSRRL = GPIO_Pin;
  }
  else  {
    GPIOx->BSRRH = GPIO_Pin;
  }
}

void LCD_Set_Data(uint8_t data){

    (((data >> 7 ) &0x01) == 1) ? DB7(1) : DB7(0);
    (((data >> 6 ) &0x01) == 1) ? DB6(1) : DB6(0);
    (((data >> 5 ) &0x01) == 1) ? DB5(1) : DB5(0);
    (((data >> 4 ) &0x01) == 1) ? DB4(1) : DB4(0);
    (((data >> 3 ) &0x01) == 1) ? DB3(1) : DB3(0);
    (((data >> 2 ) &0x01) == 1) ? DB2(1) : DB2(0);
    (((data >> 1 ) &0x01) == 1) ? DB1(1) : DB1(0);
    (((data >> 0 ) &0x01) == 1) ? DB0(1) : DB0(0);
}
void LCD_SendCommand(uint8_t data){
    RS(0);
    LCD_Set_Data(data);
    EN(1);
    delay_ms(1);//10
    EN(0);
}

void LCD_SendData(uint8_t data){
    RS(1);
    LCD_Set_Data(data);
    EN(1);
    delay_ms(1);//10
    EN(0);
}
void LCD_ini(void){
	RW(0);
	LCD_SendCommand(0x38);
	delay_ms(50);
	LCD_SendCommand(0x38);
	delay_ms(50);
	LCD_SendCommand(0x0f);
	delay_ms(50);
	LCD_SendCommand(0x01);
	delay_ms(50);
	LCD_SendCommand(0x06);
	delay_ms(50);
	LCD_SendCommand(0x02);
	delay_ms(50);
}
uint8_t LCD_Send_String(uint8_t String_Num, char* str){
	uint8_t i=0;
	uint8_t rowAdr[4] = {0x80, 0xc0, 0x94, 0xd4};
	if (String_Num == 0)
		LCD_SendCommand(0x80);
	if (String_Num == 1)
		LCD_SendCommand(0xC0);
	if (String_Num == 2)
	    LCD_SendCommand(0x94);
    if (String_Num == 3)
	    LCD_SendCommand(0xD4);
	delay_ms(1);//10

	while (str[i] != '\n' && str[i] != 0)	{		
		LCD_SendData(str[i]);	
		delay_ms(1);//10
		i++;
		if(i%20 == 0)
			LCD_SendCommand( rowAdr[++String_Num%4] );
	}
	return i;
}
