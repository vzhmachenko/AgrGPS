#include "charQueue.h"


void moveDollToBegin(queue *q){
	char *dollar = strchr(q->queue, '$');						// Ищем положение знака доллара.

	if(dollar 																			// Знак доллара нашли
	&& dollar - q->queue < 200){										// Если нашли и знак в пределах массива.
		if( dollar != q->queue){											// Знак доллара не в начале массива.
			uint16_t strl = strlen(dollar);							// Находим размер строки начиная от знака доллара.
			if(dollar - q->queue + strl > 200){					
				strl = 200 - (dollar - q->queue);
			}
			memcpy(q->queue, dollar, strl);
			memset(q->queue + strl, 0, 200 - strl);
			q->full = strl;
		}
	}
	else{
		memset(q->queue, 0, 200);					// Если нету доллара, то очищаем.
		q->full = 0;
	}
}

/* Додаем сообщение в общий массив. */
void 
push(queue *q, char *src, uint16_t size){
	if(200 - q->full > size){											// Достаточно свободных ечеек в массиве
		memcpy(q->queue + q->full, src, size);
		q->full += size;
	}
	moveDollToBegin(q);
}

uint16_t 
pop(char *dest, queue *q){
	if(!q->full){															// Если нету данных, то выходим.
		return 0;
	}

	memset(dest, 0, strlen_t);									// Очищаем выходной массив
																							
	char *dollar 	= strchr(q->queue, '$');			// Знак доллар точно в начале массива.
	char *eof 		= strchr(q->queue, '\n');			// Ищем перевод строки.

	if(dollar																		// Если есть доллар
	&& eof																			// И есть перевод каретки
	&& dollar == q->queue												// И долар в начале массива
	&& eof - q->queue < 200){										// И конец строки в пределах массива
		uint16_t sizeOf1 = eof - dollar + 1;			// Ищем размер сообщения от доллака к переводу строки.
		memcpy(dest, dollar, sizeOf1);						// Копируем сообщение в выходной массив.

		*dollar = '0';														// Затираем первый знак доллара
		moveDollToBegin(q);												// Перемещаем следующий доллар в начало.

		return sizeOf1;
	}
	else {
		return 0;
	}
}
	







void 
create(queue *new){
	new->full = 0;
	new->push  = &push;
	new->pop = &pop; 
	memset(new->queue, 0, 200);
}
