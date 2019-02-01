/*
 * gpio.h
 *
 *  Created on: Jan 28, 2019
 *      Author: valentyn
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "stm32f4xx.h"

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */


#define RW(a)   GPIO_WritePin(GPIOC, GPIO_PIN_14, (FlagStatus)a )
#define RS(a)   GPIO_WritePin(GPIOC, GPIO_PIN_13, (FlagStatus)a )
#define EN(a)   GPIO_WritePin(GPIOC, GPIO_PIN_15, (FlagStatus)a )

#define DB0(a)	 GPIO_WritePin(GPIOE, GPIO_PIN_0, (FlagStatus)a )
#define DB1(a)   GPIO_WritePin(GPIOE, GPIO_PIN_1, (FlagStatus)a )
#define DB2(a)   GPIO_WritePin(GPIOE, GPIO_PIN_2, (FlagStatus)a )
#define DB3(a)   GPIO_WritePin(GPIOE, GPIO_PIN_3, (FlagStatus)a )
#define DB4(a)   GPIO_WritePin(GPIOE, GPIO_PIN_4, (FlagStatus)a )
#define DB5(a)   GPIO_WritePin(GPIOE, GPIO_PIN_5, (FlagStatus)a )
#define DB6(a)   GPIO_WritePin(GPIOE, GPIO_PIN_6, (FlagStatus)a )
#define DB7(a)   GPIO_WritePin(GPIOE, GPIO_PIN_7, (FlagStatus)a )

void GPIO_WritePin(GPIO_TypeDef* GPIOx, 
		uint16_t GPIO_Pin, FlagStatus PinState);
void timerini2(void);
void delay_ms(uint16_t ms);
void LCD_Set_Data(uint8_t data);
void LCD_SendCommand(uint8_t data);
void LCD_SendData(uint8_t data);
void LCD_ini(void);
uint8_t LCD_Send_String(uint8_t String_Num, char* str);



#endif /* GPIO_H_ */
