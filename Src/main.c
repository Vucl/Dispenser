/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Denis Petrunin
 * @brief          : Main program body
 ******************************************************************************
*/

#include <stdint.h>
//#include "stm32f4xx.h"

#define PBA 0x40000000 // peripheral Base Address
#define PBA_AHB1 (PBA+0x20000) // AHB1 PBA

//светодиоды
#define GPIOD_BASE (PBA_AHB1+0xC00) // порт со светодиодами на ногах 12-15
#define GPIOD ((GPIO_TypeDef *) GPIOD_BASE)

//PA0 клавиша
#define GPIOA_BASE (PBA_AHB1+0x0) // порт с клавишей PA0 (без смещения)
#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)


typedef struct
{
	uint32_t MODER;//  0x00 //нужен, 00==вход 01==выход
	uint32_t OTYPER;//  0x04
	uint32_t OSPEEDR;//  0x08
	uint32_t PUPDR;//  0x0C
	uint32_t IDR;//  0x10 //нужен
	uint32_t ODR;//  0x14 //нужен
	uint32_t BSRR;//  0x18
	uint32_t LCKR;//  0x1C
	uint32_t AFRL;//  0x20
	uint32_t AFRH;//  0x24
} GPIO_TypeDef;

//#define peripheralBitBandRegionBaseAdress
//#define advancedHardwareBus1PBA

int main(void)
{
	*((uint32_t*)0x40023830)|=0b1001; // RCC_AHB1ENR GPIODEN GPIOAEN
	GPIOD->MODER |= (1<<24) | (1<<26) | (1<<28) | (1<<30); //пины leds на вывод
	//GPIOD->MODER |= 0x55000000;
	GPIOA->MODER &= ~(0b11<<0); // PA0 на вход

	GPIOD->ODR = 0xF000;
    /* Loop forever */
	while(28)
	{

		if ((GPIOA->IDR & (1<<0)) != 0){
			GPIOD->ODR |= 1<<15;
		}
		else {
			GPIOD->ODR &= ~(1<<15);
		}

		__asm("nop");
	}
}
