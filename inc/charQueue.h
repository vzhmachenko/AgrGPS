#ifndef _CHAR_QUEUEU_
#define _CHAR_QUEUEU_

#include <string.h>
#include <stdlib.h>
#include "stm32f4xx.h"

#define strlen_t 200            //Максимальная длина строки
#define strlen_r 80
#define strlen 80

typedef struct {
	char queue[strlen_t];
	uint16_t empty;
	uint16_t full;
	void (*push)(void*, char*, uint16_t);
	uint16_t (*pop)(char*, void*);
   	void (*create)(void*);

} queue;

void strAllcpy(char *to, char *from, uint16_t size);
uint16_t findEOS(queue *q);
void push(queue *q, char *str, uint16_t size);
uint16_t pop(char *to, queue *q);
void create(queue *new);


#endif /* GPIO_H_ */