/* SRAM tester
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stm32f1xx.h>
#include <stdint.h>
#include <system_stm32f1xx.h>
#include <stm32f1xx_ll_utils.h>
#include <math.h>

#include "global.h"

uint32_t sys_time = 0;
uint32_t sys_time_seconds = 0;


int main(void) {
    /* Get all the good clocks and PLLs on this thing up and running. We're running from an external 8MHz crystal.
     * which we're passing through PREDIV1 to get 8MHz into the main PLL. The main PLL multiplies this by 9 to get 72MHz.
     *
     * APB2 is run at full 72MHz, APB1 at 36MHz. The AHB is also run at full 72MHz.
     *
     * The busses are all run directly from these 30 MHz because why not.
     *
     * Be careful in mucking around with this code since you can kind of semi-brick the chip if you do it wrong.
     */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));

    // HSE ready, let's configure the PLL
    // PLLMUL: 9x (0b0111), select PREDIV1 as PLL input, set APB2/AHB to full 72MHz, APB1 to 36MHz (prediv 2)
    RCC->CFGR = (0b0111<<RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLSRC | (0b100<RCC_CFGR_PPRE1_Pos) | RCC_CFGR_USBPRE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));

    RCC->CFGR |= (0b10<<RCC_CFGR_SW_Pos);

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */


    /* Enable all the periphery we need */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* Configure all the GPIOs */
    GPIOA->CRL = 

    GPIOB->CRH &= ~0x000000ff;
    GPIOB->CRH |=  0x00000011;

    /* Idly loop around, occassionally disfiguring some integers. */
    while (42) {
        for (unsigned int i=0; i<1000000; i++) asm volatile("nop");
        GPIOB->BSRR = 0x02000100;
        for (unsigned int i=0; i<1000000; i++) asm volatile("nop");
        GPIOB->BSRR = 0x01000200;
    }
}

/* Misc IRQ handlers */
void NMI_Handler(void) {
}

void HardFault_Handler(void) {
    for(;;);
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    static int n = 0;
    sys_time++;
    if (n++ == 1000) {
        n = 0;
        sys_time_seconds++;
    }
}

/* Misc stuff for nostdlib linking */
void _exit(int status) { (void)status; while (23); }
void *__bss_start__;
void *__bss_end__;
int __errno;

