#include "charQueue.h"

void strAllcpy(char *to, char *from, uint16_t size){
	for(uint16_t i = 0; i < size; i++)
		*(to + i) = *(from + i);
}
uint16_t findEOS(queue *q){
	for(uint16_t i = 0; i < q->full; i++)
		if(*(q->queue+i) == '\n')
			return i;
	return 0;
}
void push(queue *q, char *str, uint16_t size){
	char temp[2];
	if(size < q->empty){
		strAllcpy(q->queue + q->full, str, size);
		q->full += size;
		q->empty -= size;
	}
	
}
uint16_t pop(char *to, queue *q){
	if(q->full > 0 && findEOS(q)){
		uint16_t temp = findEOS(q);
		strAllcpy(to, q->queue, temp);
	
		strAllcpy(q->queue, q->queue+temp+1, 
			q->full-findEOS(q));
		q->full = q->full - temp -1;
		q->empty = q->empty + temp + 1;
		return temp;
	}
}
void create(queue *new){
	new->full = 0;
	new->empty = strlen_t;
	new->push  = &push;
	new->pop = &pop; 
}