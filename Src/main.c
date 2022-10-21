/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Denis Petrunin
 * @brief          : Main program body
 ******************************************************************************
*/

#include <stdint.h>
//#include "stm32f407xx.h"

#define PBA 0x40000000 // peripheral Base Address
#define PBA_AHB1 (PBA+0x20000) // AHB1 PBA

//светодиоды
//#define GPIOD_BASE (PBA_AHB1+0xC00) // порт со светодиодами на ногах 12-15
#define GPIOD ((GPIO_TypeDef *) GPIOD_BASE)

//PA0 клавиша
//#define GPIOA_BASE (PBA_AHB1+0x0) // порт с клавишей PA0 (без смещения)
#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)

//настройка частоты
#define HCLK
#define AHB1


/*
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
*/

//#define peripheralBitBandRegionBaseAdress
//#define advancedHardwareBus1PBA

int main(void)
{
	FLASH->ACR = 0x101; //flash latency
	//RCC->CR |= RCC_CR_HSEON;
	RCC->CR |= 1<<0;  //устанавливаем 0-йбит для включения HSI
	while (!(RCC->CR & (1<<1))); //читаем сr ready пока не будет готов
	RCC->PLLCFGR |= 8<<0; //PLLM
	RCC->PLLCFGR |= 150<<6; //PLLN
	RCC->PLLCFGR |= 2<<16; //PLLP
	RCC->PLLCFGR |= 0b00<<22; //PLLSRC

	RCC->CFGR |= 0b00; // system clock switch -> HSI
	RCC->CFGR |= (0x2<<2); // PLL_P selected as system clock

	RCC->CFGR |= (0x1<<4); // предделитель AHB в HPRE
	RCC->CFGR |= (0x4<<10); // предделитель APB1 в PPRE1
	RCC->CFGR |= (0x2<<13); // предделитель APB2 в PPRE2

	//RCC->AHB1ENR |= 0b11111111; //вкл все порты ввода-вывода
	RCC->AHB1ENR |= 0b11<<21; //вкл DMA, на всякий :|

	SysTick_Config(SysTicks);

	*((uint32_t*)0x40023830)|=0b1001; // RCC_AHB1ENR GPIODEN GPIOAEN
	GPIOD->MODER |= (1<<24) | (1<<26) | (1<<28) | (1<<30); //пины leds на вывод
	//GPIOD->MODER |= 0x55000000;
	GPIOA->MODER &= ~(0b11<<0); // PA0 на вход

	GPIOD->ODR = 0xF000;
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

void SysTick_Handler(void)
{
	//SysTick Interrupt handler
}

