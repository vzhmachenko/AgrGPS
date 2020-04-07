#ifndef _CHAR_QUEUEU_
#define _CHAR_QUEUEU_

#include <string.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "periph.h"


typedef struct {
	char queue[strlen_t];
	uint16_t full;
	void (*push)	 (void*, char*, uint16_t);
	uint16_t (*pop)(char*, void*);
	void (*create) (void*);

} queue;

void      push	 (queue *q, char *str, uint16_t size);
void      create (queue *new);
uint16_t  pop	(char *to, queue *q);
void 			moveDollToBegin(queue *q);


#endif /* GPIO_H_ */
