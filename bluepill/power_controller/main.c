#include <string.h>
#include "stm8s.h"

static volatile short int delay;

int main() {
	unsigned short delay = 0;
	unsigned char c;
	CLK->CKDIVR = 0x00; /* Divide by 1, f=16MHz */
	CLK->PCKENR1 = 0x88; /* Enable TIM1, UART1 */

	GPIOD->DDR = 0x20; /* TX */
	GPIOD->CR1 = 0x60; /* push-pull/pullup */
	GPIOB->DDR = 0x20; /* LED */
	GPIOB->CR1 = 0x20; /* push-pull */
	GPIOB->ODR = 0x00; /* enable board */

	UART1->CR2 = UART1_CR2_REN;
	UART1->CR3 = UART1_STOPBITS_1;
	/* Set baud rate to 9600Bd */
	UART1->BRR2 = 0x03;
	UART1->BRR1 = 0x68;

	/* Prescaler 16000, 1ms per tick */
	TIM1->PSCRH = 62;
	TIM1->PSCRL = 127;
	TIM1->IER   = 0x01; /* UIE */
	TIM1->ARRH  = 0;
	TIM1->ARRL  = 1;
	TIM1->CR1   = 0x09;

	rim();
	do {
		while (!(UART1->SR & UART1_FLAG_RXNE));
		c = UART1->DR;

		if (c == 0) {
			delay -= 1;
			TIM1->ARRH  = delay>>8;
			TIM1->ARRL  = delay&0xff;
			GPIOB->ODR |= 0x20;
			TIM1->CR1   = 0x09;
		} else {
			delay = (delay<<8) | (c-1);
		}
	} while(1);
}

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11){
	GPIOB->ODR &= ~0x20;
	TIM1->SR1 = 0;
}
