#include "periph.h"
#include "charQueue.h"

extern char recString0[strlen_r];
extern char recString1[strlen_r];
extern char toBlue[strlen_t];

//Configure pins for LED, LCD, Keyboard
void gpio_ini(void){
	RCC->AHB1ENR =  RCC_AHB1ENR_GPIOBEN
				| RCC_AHB1ENR_GPIOCEN
				| RCC_AHB1ENR_GPIODEN
				| RCC_AHB1ENR_GPIOEEN;

	//LED bits configurating
	//PD1--right corner
    GPIOD->MODER    = 0x55555554;	//Output mode, PD0 - in(not used)
    GPIOD->OTYPER   = 0x0;			//Output push-pull
    GPIOD->OSPEEDR  = 0x55555555;	//Medium speed

	//LCD 8-bus
	//PE0->DB0, PE7->DB7, PC13->RS, PC14->RW, PC15->EN
	GPIOE->MODER 	= 0x00005555;	//Output mode
	GPIOE->OTYPER	= 0x0;			//Output push-pull
	GPIOE->OSPEEDR	= 0x00005555;	//Medium speed
	GPIOC->MODER	= 0x54000000;	//Output mode
	GPIOC->OTYPER	= 0x0;			//Output push-pull
	GPIOC->OSPEEDR 	= 0x54000000;	//Medium speed

	// Keyboard in: pb2, pe8, pe10, pe12
	GPIOB->PUPDR	= 0x20;			//Pull-down
	GPIOE->PUPDR	= 0x02220000;	//Pull-down
	// Keyboard out: pe14, pb12, pb10, pb14
	GPIOE->MODER	|= 0x10000000;	//Output mode
	GPIOE->OSPEEDR	|= 0x10000000;	//Medium speed
	GPIOB->MODER	= 0x11100000;	//Output mode
	GPIOB->OSPEEDR	= 0x11100000;	//Medium speed
	GPIOB->OTYPER	= 0;			//Output push-pull
}
void RCC_Init(void){
	FLASH->ACR =  FLASH_ACR_LATENCY_5WS	//Flash latency = 5
				| FLASH_ACR_PRFTEN
				| FLASH_ACR_ICEN
				| FLASH_ACR_DCEN;
	RCC->CFGR = 0;						//Обнуление перед настройкой
	RCC->CFGR 	|= 0b100 << 13			//APB2
				| 0b101 << 10			//APB1
				| 0b0000 << 4;			//AHB
	RCC->CR = 1 << 16;					//HSE ON
	while( (((RCC->CR) >> 17) & 0x01) == 0) ; //Пока не включен режим HSE, ждем
	RCC->PLLCFGR = 1 << 22			//PLL HSE oscillator
				| 0b000100			//PLLM = 4
				| 0b10101000 << 6 	//PLLN = 168
				| 0b00 << 16		//PLLP = 2
				| 7 << 24;			//PLLQ = 7
	RCC->CR |= 1 << 24;				//PLL ON
	while( (((RCC->CR) >> 25) & 0x01) == 0)	; //Ждем, пока не заблокируется PLL
	RCC->CFGR &= ~RCC_CFGR_SW; 		//Очищаем бит
	RCC->CFGR |= RCC_CFGR_SW_PLL;   //Устанавливаем
	while((RCC->CFGR & RCC_CFGR_SW_PLL) != RCC_CFGR_SW_PLL) ;
}
//GPS
void USART2_init(void){
	//PA2     ------> USART2_TX
    //PA3     ------> USART2_RX 
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER	|= 0xA0;			//Alternate function mode
	GPIOA->PUPDR	= 0xA0;			//Pullup
	GPIOA->OSPEEDR	= 0xF0;			//Very high speed
	GPIOA->AFR[0]	= 0x7700;		//Alternate function 7

	RCC->APB1ENR	|= RCC_APB1ENR_USART2EN;//USART2EN
	USART2->CR1		=	USART_CR1_RE
					|	USART_CR1_TE;
	USART2->CR2		= 0x0;			//1 Stop bit
	USART2->CR3		= 0x0;
	USART2->BRR		= 0x1117;

    //NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR1 |= USART_CR1_UE
				|  USART_CR1_RXNEIE;
}

//BlueTooth
void USART6_init(void){
	//PC6     ------> USART6_TX
    //PC7     ------> USART6_RX 
	GPIOC->MODER	= 0xA800A000;			//Alternate function mode
	GPIOC->PUPDR	|= 0xA000;			//Pullup
	GPIOC->OSPEEDR	|= 0xF000;			//Very high speed
	GPIOC->AFR[0]	= 0x88000000;		//Alternate function 8

	RCC->APB2ENR	|= RCC_APB2ENR_USART6EN;			//USART6EN
	USART6->CR1		=	USART_CR1_RE
					|	USART_CR1_TE;
	USART6->CR2		= 0x0;			//1 Stop bit
	USART6->CR3		= 0x0;
	USART6->BRR		= 0x222e;
/*Enable transmit what receive uncomment it*/
//    NVIC_EnableIRQ(USART6_IRQn);

    USART6->CR1 |= USART_CR1_UE
				|  USART_CR1_RXNEIE;
}

void dma1ini(void) {
    USART2->CR1 &= ~(USART_CR1_RXNEIE   //rx interrupt
                    | USART_CR1_UE);     //USART2 enable
    USART2->CR3 |= USART_CR3_DMAR;		//Enable DMA receiver for USART2
    USART2->CR1 |= USART_CR1_UE;		//запуск uart

	RCC->AHB1ENR |=	RCC_AHB1ENR_DMA1EN; // вкл тактир дма
	DMA1_Stream5->CR = 0;               //Обнуляем значение регистра
	DMA1_Stream5->PAR = (uint32_t)&USART2->DR;          //Адрес ОТКУДА
	DMA1_Stream5->M0AR = (uint32_t)&recString0; //Адрес КУДA
	DMA1_Stream5->M1AR = (uint32_t)&recString1;	//Адрес КУДA
	DMA1_Stream5->NDTR = strlen_r;              // Количество даних для передачи 
	DMA1_Stream5->CR |= DMA_SxCR_CHSEL_2	//8 - channel 4
					 |  DMA_SxCR_PL_1		//1 - priority level High
					 |  DMA_SxCR_MINC		//4 - memory address pointer increment
					 |  DMA_SxCR_CIRC		//circular mode
					 | 	DMA_SxCR_TCIE   	//Transfer complete interrupt enable    
					 |  DMA_SxCR_DBM;		//Double buffer mode
	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;        //Сбросить бит прервания
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);      //вкл обработку прер для
    NVIC_SetPriority(DMA1_Stream5_IRQn, 15);
      DMA1_Stream5->CR |= DMA_SxCR_EN;		//DMA -> EN

}

void dma2ini(void) {
	//DMA Stream 7, Channel 5
    USART6->CR1 &= ~(USART_CR1_RXNEIE   //rx interrupt
                    | USART_CR1_UE);     //USART2 enable
    USART6->CR3 |= USART_CR3_DMAT;		//Enable DMA transmitter for USART2
    USART6->CR1 |= USART_CR1_UE;		//запуск uart

	RCC->AHB1ENR |=	RCC_AHB1ENR_DMA2EN; // вкл тактир дма2
	DMA2_Stream7->CR = 0;               //Обнуляем значение регистра
	DMA2_Stream7->PAR = (uint32_t)&USART6->DR;          //Адрес КУДА
	DMA2_Stream7->M0AR = (uint32_t)&toBlue; //Адрес OTКУДA
	DMA2_Stream7->NDTR = strlen_t;              // Количество даних для передачи 
	DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2 	// channel 5
					 |  DMA_SxCR_CHSEL_0	// channel 5
					 |  DMA_SxCR_PL_1		//1 - priority level High
					 |  DMA_SxCR_MINC		//4 - memory address pointer increment
//					 |  DMA_SxCR_CIRC		//circular mode
					 |  DMA_SxCR_DIR_0		//peripheral to Memory
					 | 	DMA_SxCR_TCIE;   	//Transfer complete interrupt enable    
//					 |  DMA_SxCR_PFCTRL;	//Peripheral flow control
//					 |  DMA_SxCR_DBM;		//Double buffer mode
	DMA2->HIFCR |= DMA_HIFCR_CTCIF7;        //Сбросить бит прервания
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);      //вкл обработку прер для
//    DMA2_Stream7->CR |= DMA_SxCR_EN;		//DMA -> EN
	NVIC_SetPriority(DMA2_Stream7_IRQn, 16);

}
