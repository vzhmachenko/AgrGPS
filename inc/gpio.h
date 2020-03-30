#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4xx.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//#define NULL    ( (void *) 0)

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

#define RW(a)    GPIO_WritePin(GPIOC, GPIO_PIN_14, (FlagStatus)a )
#define RS(a)    GPIO_WritePin(GPIOC, GPIO_PIN_13, (FlagStatus)a )
#define EN(a)    GPIO_WritePin(GPIOC, GPIO_PIN_15, (FlagStatus)a )

// Адреса начальных символов строк
static const uint8_t lineAddr[4] = {0x80, 0xc0, 0x94, 0xd4};

/*! Структура для передачи строки дисплею
 *  посредством очереди. */
typedef struct {
	uint8_t lineNumber;
	char string[30];
} lineParam;

/*! Устанавливаем значение пина. */
void GPIO_WritePin(GPIO_TypeDef*	GPIOx, 
									 uint16_t				GPIO_Pin, 
									 FlagStatus			PinState);

/*! Установка таймера 2.*/
void timerini2(void);

/* Задержка, работающая на 2 таймере. */
void delay_ms(uint16_t ms);

/*! Запись данных на ножки дисплея. */
void LCD_Set_Data(uint8_t data);

/* Отправляем команду на дисплей. */
void LCD_SendCommandOrData(uint8_t data, uint8_t command);

/*! Процедура инициализации дисплея. */
void LCD_ini(void);

/* Отправляем строку на дисплей. */
void LCD_Send_String(uint8_t String_Num, char *str);

/* Вывод на дисплей значение регистра (32bit-value). */
void regToDisplay(uint32_t reg, int8_t strNum);

/* Вывод на дисплей числа с запятой. */
void doubleToDisplay(double num, uint8_t strNum);

/* Инициализация структуры, для представления выводан на дисплей,
 * посредством очереди freeRTOS. */
void initLCDstruct(lineParam* line, uint8_t string_num, char *str);

/* Выводим строку на дисплей через очередь и задачу. */
void strToDisplay(QueueHandle_t destQueue, uint8_t strNum, char* str);

/* Додаем ДаблЧисло в очередь на вывод. */
void addToQueue_doubleToDisplay(QueueHandle_t destQueue, double num, 
																uint8_t strNum);

#endif /* GPIO_H_ */
